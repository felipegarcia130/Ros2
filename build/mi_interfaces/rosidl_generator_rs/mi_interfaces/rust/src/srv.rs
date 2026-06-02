#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};




// Corresponds to mi_interfaces__srv__Saludo_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Saludo_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub nombre: std::string::String,

}



impl Default for Saludo_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::Saludo_Request::default())
  }
}

impl rosidl_runtime_rs::Message for Saludo_Request {
  type RmwMsg = super::srv::rmw::Saludo_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        nombre: msg.nombre.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        nombre: msg.nombre.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      nombre: msg.nombre.to_string(),
    }
  }
}


// Corresponds to mi_interfaces__srv__Saludo_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Saludo_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub mensaje: std::string::String,

}



impl Default for Saludo_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::Saludo_Response::default())
  }
}

impl rosidl_runtime_rs::Message for Saludo_Response {
  type RmwMsg = super::srv::rmw::Saludo_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        mensaje: msg.mensaje.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        mensaje: msg.mensaje.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      mensaje: msg.mensaje.to_string(),
    }
  }
}






#[link(name = "mi_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__mi_interfaces__srv__Saludo() -> *const std::ffi::c_void;
}

// Corresponds to mi_interfaces__srv__Saludo
#[allow(missing_docs, non_camel_case_types)]
pub struct Saludo;

impl rosidl_runtime_rs::Service for Saludo {
    type Request = Saludo_Request;
    type Response = Saludo_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__mi_interfaces__srv__Saludo() }
    }
}


