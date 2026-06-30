#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to lidar_msgs__msg__LidarSweep

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct LidarSweep {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub angles: Vec<f64>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub distances: Vec<f64>,

}



impl Default for LidarSweep {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::LidarSweep::default())
  }
}

impl rosidl_runtime_rs::Message for LidarSweep {
  type RmwMsg = super::msg::rmw::LidarSweep;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        angles: msg.angles.into(),
        distances: msg.distances.into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        angles: msg.angles.as_slice().into(),
        distances: msg.distances.as_slice().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      angles: msg.angles
          .into_iter()
          .collect(),
      distances: msg.distances
          .into_iter()
          .collect(),
    }
  }
}


