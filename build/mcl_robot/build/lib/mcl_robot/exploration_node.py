"""
motion_calibration.py
─────────────────────────────────────────────────────────────────────────────
Nodo ROS2 de calibración experimental para estimar los parámetros αᵢ del
modelo de ruido de odometría diferencial (Thrun et al., §5.3).

MODELO MATEMÁTICO
─────────────────
Para un robot diferencial, el error de odometría entre paso k-1 y k se
descompone en tres primitivas:
    δ_rot1  = atan2(y_k - y_{k-1}, x_k - x_{k-1}) - θ_{k-1}  (giro inicial)
    δ_trans = ‖(x_k-x_{k-1}, y_k-y_{k-1})‖                   (traslación)
    δ_rot2  = θ_k - θ_{k-1} - δ_rot1                          (giro final)

Las varianzas de ruido son (Thrun §5.3.2):
    σ²_rot1  = α₁·δ_rot1²  + α₂·δ_trans²
    σ²_trans = α₃·δ_trans² + α₄·(δ_rot1² + δ_rot2²)
    σ²_rot2  = α₁·δ_rot2²  + α₂·δ_trans²

USO
───
1. Conectar el robot y asegurarse de que /odom y /ground_truth (o /amcl_pose)
   estén publicando.
2. Mover el robot por la trayectoria de calibración (avance recto, giro en
   sitio, curva) al menos 10 repeticiones.
3. Correr este nodo:
       ros2 run <pkg> motion_calibration
4. Al terminar (Ctrl+C) imprime los αᵢ y los guarda en motion_params.yaml.
5. Copiar los valores al inicio de mcl_node_improved.py.

Si no hay ground truth, se usa el modo "ciclo cerrado": el robot regresa al
origen y el error acumulado se usa para estimar αᵢ vía mínimos cuadrados.
"""

import math
import yaml
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class MotionCalibrationNode(Node):
    """
    Recopila pares (odometría_delta, ground_truth_delta) y estima αᵢ por
    regresión lineal ordinaria (OLS) en forma matricial.

    Si solo está disponible /odom (sin ground truth), usa el modo de
    varianza muestral sobre múltiples repeticiones de la misma maniobra.
    """

    def __init__(self):
        super().__init__('motion_calibration')

        self.odom_poses: list[np.ndarray] = []   # historial de poses odom
        self.gt_poses:   list[np.ndarray] = []   # historial ground truth

        # Acumuladores para OLS:
        # Cada fila: [δ_rot1², δ_trans², δ_rot2²] → target: error²
        self._rows_rot:   list[np.ndarray] = []  # para α₁, α₂
        self._rows_trans: list[np.ndarray] = []  # para α₃, α₄
        self._y_rot1:     list[float]       = []
        self._y_rot2:     list[float]       = []
        self._y_trans:    list[float]       = []

        self._last_odom: np.ndarray | None = None
        self._last_gt:   np.ndarray | None = None
        self._n_steps = 0

        self.sub_odom = self.create_subscription(
            Odometry, '/odom', self._cb_odom, 10)
        self.sub_gt = self.create_subscription(
            Odometry, '/ground_truth', self._cb_gt, 10)

        self.get_logger().info(
            'Calibración iniciada. Mueve el robot y presiona Ctrl+C al terminar.'
        )

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _cb_odom(self, msg: Odometry):
        pose = self._msg_to_pose(msg)
        if self._last_odom is None:
            self._last_odom = pose
            return
        delta = self._compute_delta(self._last_odom, pose)
        self._last_odom = pose
        self._odom_deltas_buf = getattr(self, '_odom_deltas_buf', [])
        self._odom_deltas_buf.append(delta)

    def _cb_gt(self, msg: Odometry):
        pose = self._msg_to_pose(msg)
        if self._last_gt is None:
            self._last_gt = pose
            return
        delta_gt = self._compute_delta(self._last_gt, pose)
        self._last_gt = pose

        buf = getattr(self, '_odom_deltas_buf', [])
        if not buf:
            return
        delta_odom = buf.pop(0)  # par más reciente

        self._accumulate(delta_odom, delta_gt)
        self._n_steps += 1
        if self._n_steps % 50 == 0:
            self.get_logger().info(f'Pares recopilados: {self._n_steps}')

    # ── Matemáticas ──────────────────────────────────────────────────────────

    @staticmethod
    def _msg_to_pose(msg: Odometry) -> np.ndarray:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        theta = 2.0 * math.atan2(q.z, q.w)
        return np.array([x, y, theta], dtype=np.float64)

    @staticmethod
    def _compute_delta(p0: np.ndarray, p1: np.ndarray) -> np.ndarray:
        """
        Devuelve (δ_rot1, δ_trans, δ_rot2) según Thrun §5.3.
        """
        dx = p1[0] - p0[0]
        dy = p1[1] - p0[1]
        d_trans = math.sqrt(dx**2 + dy**2)
        if d_trans < 1e-6:
            d_rot1 = 0.0
        else:
            d_rot1 = math.atan2(dy, dx) - p0[2]
            d_rot1 = (d_rot1 + math.pi) % (2 * math.pi) - math.pi
        d_rot2 = (p1[2] - p0[2]) - d_rot1
        d_rot2 = (d_rot2 + math.pi) % (2 * math.pi) - math.pi
        return np.array([d_rot1, d_trans, d_rot2], dtype=np.float64)

    def _accumulate(self, delta_odom: np.ndarray, delta_gt: np.ndarray):
        """
        Acumula un par (odom, gt) en los buffers de regresión.

        El error de cada primitiva es:
            e_rot1  = delta_gt[0] - delta_odom[0]
            e_trans = delta_gt[1] - delta_odom[1]
            e_rot2  = delta_gt[2] - delta_odom[2]

        Según el modelo de Thrun:
            σ²_rot1  = α₁·δ_rot1²  + α₂·δ_trans²
            σ²_trans = α₃·δ_trans² + α₄·(δ_rot1² + δ_rot2²)
            σ²_rot2  = α₁·δ_rot2²  + α₂·δ_trans²

        Esto es un sistema lineal en [α₁,α₂] y [α₃,α₄] por separado.
        Aproximamos σ² ≈ e² (estimador de momento de orden 2).
        """
        e = delta_gt - delta_odom
        r1, t, r2 = delta_odom

        # Para α₁, α₂: predice e_rot1² y e_rot2²
        self._rows_rot.append(np.array([r1**2, t**2]))
        self._y_rot1.append(e[0]**2)
        self._rows_rot.append(np.array([r2**2, t**2]))
        self._y_rot2.append(e[2]**2)

        # Para α₃, α₄: predice e_trans²
        self._rows_trans.append(np.array([t**2, r1**2 + r2**2]))
        self._y_trans.append(e[1]**2)

    # ── Estimación final ─────────────────────────────────────────────────────

    def estimate_alphas(self) -> dict[str, float]:
        """
        Resuelve OLS con restricción de no-negatividad (NNLS):
            min ‖Aα - y‖²  s.t. α ≥ 0

        Referencia: scipy.optimize.nnls (Lawson & Hanson, 1974).
        """
        from scipy.optimize import nnls

        if len(self._rows_rot) < 10:
            self.get_logger().warn(
                'Muy pocos pares para estimar. Usando valores por defecto.')
            return {'alpha1': 0.1, 'alpha2': 0.1,
                    'alpha3': 0.05, 'alpha4': 0.05}

        # Bloques de rotación
        A_rot = np.vstack(self._rows_rot)
        y_rot = np.array(self._y_rot1 + self._y_rot2)
        alpha_12, _ = nnls(A_rot, y_rot)

        # Bloque de traslación
        A_trans = np.vstack(self._rows_trans)
        y_trans = np.array(self._y_trans)
        alpha_34, _ = nnls(A_trans, y_trans)

        result = {
            'alpha1': float(alpha_12[0]),
            'alpha2': float(alpha_12[1]),
            'alpha3': float(alpha_34[0]),
            'alpha4': float(alpha_34[1]),
        }
        return result

    def save_and_report(self):
        alphas = self.estimate_alphas()
        self.get_logger().info('═' * 60)
        self.get_logger().info('RESULTADO DE CALIBRACIÓN')
        self.get_logger().info('═' * 60)
        for k, v in alphas.items():
            self.get_logger().info(f'  {k} = {v:.6f}')
        self.get_logger().info('─' * 60)
        self.get_logger().info(
            'Interpretación:\n'
            '  α₁, α₂ controlan el ruido de rotación\n'
            '  α₃, α₄ controlan el ruido de traslación\n'
            '  Valores típicos para encoders de bajo costo: 0.02 – 0.2'
        )

        # Calcular la covarianza equivalente para un movimiento típico
        # (avance 0.1m con giro pequeño 0.05rad) — para inicializar RAM
        dt, dr = 0.1, 0.05
        sigma_rot   = math.sqrt(alphas['alpha1']*dr**2 + alphas['alpha2']*dt**2)
        sigma_trans = math.sqrt(alphas['alpha3']*dt**2 + alphas['alpha4']*dr**2)
        self.get_logger().info(
            f'\nCovarianza típica (dt=0.1m, dr=0.05rad):\n'
            f'  σ_row (fila/y en píxeles) ≈ {sigma_trans:.4f} m → {sigma_trans/0.05:.1f} px\n'
            f'  σ_col (col/x en píxeles)  ≈ {sigma_trans:.4f} m → {sigma_trans/0.05:.1f} px\n'
            f'  σ_theta                   ≈ {sigma_rot:.4f} rad\n'
            f'\nUsar en mcl_node_improved.py:\n'
            f'  S0_scale = np.array([{sigma_trans/0.05:.3f}, {sigma_trans/0.05:.3f}, {sigma_rot:.4f}])'
        )

        # Guardar YAML
        with open('/tmp/motion_params.yaml', 'w') as f:
            yaml.dump({'motion_noise': alphas}, f)
        self.get_logger().info('\nGuardado en /tmp/motion_params.yaml')
        return alphas


def main():
    rclpy.init()
    node = MotionCalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_and_report()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()