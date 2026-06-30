#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "lidar_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__lidar_msgs__msg__LidarSweep() -> *const std::ffi::c_void;
}

#[link(name = "lidar_msgs__rosidl_generator_c")]
extern "C" {
    fn lidar_msgs__msg__LidarSweep__init(msg: *mut LidarSweep) -> bool;
    fn lidar_msgs__msg__LidarSweep__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<LidarSweep>, size: usize) -> bool;
    fn lidar_msgs__msg__LidarSweep__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<LidarSweep>);
    fn lidar_msgs__msg__LidarSweep__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<LidarSweep>, out_seq: *mut rosidl_runtime_rs::Sequence<LidarSweep>) -> bool;
}

// Corresponds to lidar_msgs__msg__LidarSweep
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct LidarSweep {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub angles: rosidl_runtime_rs::Sequence<f64>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub distances: rosidl_runtime_rs::Sequence<f64>,

}



impl Default for LidarSweep {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !lidar_msgs__msg__LidarSweep__init(&mut msg as *mut _) {
        panic!("Call to lidar_msgs__msg__LidarSweep__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for LidarSweep {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { lidar_msgs__msg__LidarSweep__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { lidar_msgs__msg__LidarSweep__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { lidar_msgs__msg__LidarSweep__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for LidarSweep {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for LidarSweep where Self: Sized {
  const TYPE_NAME: &'static str = "lidar_msgs/msg/LidarSweep";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__lidar_msgs__msg__LidarSweep() }
  }
}


