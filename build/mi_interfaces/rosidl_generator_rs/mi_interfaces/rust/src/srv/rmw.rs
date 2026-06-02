#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



#[link(name = "mi_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__mi_interfaces__srv__Saludo_Request() -> *const std::ffi::c_void;
}

#[link(name = "mi_interfaces__rosidl_generator_c")]
extern "C" {
    fn mi_interfaces__srv__Saludo_Request__init(msg: *mut Saludo_Request) -> bool;
    fn mi_interfaces__srv__Saludo_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Saludo_Request>, size: usize) -> bool;
    fn mi_interfaces__srv__Saludo_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Saludo_Request>);
    fn mi_interfaces__srv__Saludo_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Saludo_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<Saludo_Request>) -> bool;
}

// Corresponds to mi_interfaces__srv__Saludo_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Saludo_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub nombre: rosidl_runtime_rs::String,

}



impl Default for Saludo_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !mi_interfaces__srv__Saludo_Request__init(&mut msg as *mut _) {
        panic!("Call to mi_interfaces__srv__Saludo_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Saludo_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { mi_interfaces__srv__Saludo_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { mi_interfaces__srv__Saludo_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { mi_interfaces__srv__Saludo_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Saludo_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Saludo_Request where Self: Sized {
  const TYPE_NAME: &'static str = "mi_interfaces/srv/Saludo_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__mi_interfaces__srv__Saludo_Request() }
  }
}


#[link(name = "mi_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__mi_interfaces__srv__Saludo_Response() -> *const std::ffi::c_void;
}

#[link(name = "mi_interfaces__rosidl_generator_c")]
extern "C" {
    fn mi_interfaces__srv__Saludo_Response__init(msg: *mut Saludo_Response) -> bool;
    fn mi_interfaces__srv__Saludo_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Saludo_Response>, size: usize) -> bool;
    fn mi_interfaces__srv__Saludo_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Saludo_Response>);
    fn mi_interfaces__srv__Saludo_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Saludo_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<Saludo_Response>) -> bool;
}

// Corresponds to mi_interfaces__srv__Saludo_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Saludo_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub mensaje: rosidl_runtime_rs::String,

}



impl Default for Saludo_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !mi_interfaces__srv__Saludo_Response__init(&mut msg as *mut _) {
        panic!("Call to mi_interfaces__srv__Saludo_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Saludo_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { mi_interfaces__srv__Saludo_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { mi_interfaces__srv__Saludo_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { mi_interfaces__srv__Saludo_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Saludo_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Saludo_Response where Self: Sized {
  const TYPE_NAME: &'static str = "mi_interfaces/srv/Saludo_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__mi_interfaces__srv__Saludo_Response() }
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


