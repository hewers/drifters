pub mod drifters_ {
    pub mod v1_ {
        /// An instant in GPS time. Differences within a run stay exact; the week number
        /// is not modulo-1024.
        #[derive(Debug, Default, PartialEq, Clone, Copy)]
        pub struct GpsTime {
            pub r#week: u32,
            /// Time of week, seconds in [0, 604800).
            pub r#tow_s: f64,
        }
        impl GpsTime {
            /// Return a reference to `week`
            #[inline]
            pub fn r#week(&self) -> &u32 {
                &self.r#week
            }
            /// Return a mutable reference to `week`
            #[inline]
            pub fn mut_week(&mut self) -> &mut u32 {
                &mut self.r#week
            }
            /// Set the value of `week`
            #[inline]
            pub fn set_week(&mut self, value: u32) -> &mut Self {
                self.r#week = value.into();
                self
            }
            /// Builder method that sets the value of `week`. Useful for initializing the message.
            #[inline]
            pub fn init_week(mut self, value: u32) -> Self {
                self.r#week = value.into();
                self
            }
            /// Return a reference to `tow_s`
            #[inline]
            pub fn r#tow_s(&self) -> &f64 {
                &self.r#tow_s
            }
            /// Return a mutable reference to `tow_s`
            #[inline]
            pub fn mut_tow_s(&mut self) -> &mut f64 {
                &mut self.r#tow_s
            }
            /// Set the value of `tow_s`
            #[inline]
            pub fn set_tow_s(&mut self, value: f64) -> &mut Self {
                self.r#tow_s = value.into();
                self
            }
            /// Builder method that sets the value of `tow_s`. Useful for initializing the message.
            #[inline]
            pub fn init_tow_s(mut self, value: f64) -> Self {
                self.r#tow_s = value.into();
                self
            }
        }
        impl ::micropb::MessageDecode for GpsTime {
            fn decode<IMPL_MICROPB_READ: ::micropb::PbRead>(
                &mut self,
                decoder: &mut ::micropb::PbDecoder<IMPL_MICROPB_READ>,
                len: usize,
            ) -> Result<(), ::micropb::DecodeError<IMPL_MICROPB_READ::Error>> {
                use ::micropb::{PbBytes, PbString, PbVec, PbMap, FieldDecode};
                let before = decoder.bytes_read();
                while decoder.bytes_read() - before < len {
                    let tag = decoder.decode_tag()?;
                    match tag.field_num() {
                        0 => return Err(::micropb::DecodeError::ZeroField),
                        1u32 => {
                            let mut_ref = &mut self.r#week;
                            {
                                let val = decoder.decode_varint32()?;
                                let val_ref = &val;
                                if *val_ref != 0 {
                                    *mut_ref = val as _;
                                }
                            };
                        }
                        2u32 => {
                            let mut_ref = &mut self.r#tow_s;
                            {
                                let val = decoder.decode_double()?;
                                let val_ref = &val;
                                if *val_ref != 0.0 {
                                    *mut_ref = val as _;
                                }
                            };
                        }
                        _ => {
                            decoder.skip_wire_value(tag.wire_type())?;
                        }
                    }
                }
                Ok(())
            }
        }
        impl ::micropb::MessageEncode for GpsTime {
            const MAX_SIZE: ::core::result::Result<usize, &'static str> = 'msg: {
                let mut max_size = 0;
                match ::micropb::const_map!(
                    ::core::result::Result::Ok(5usize), | size | size + 1usize
                ) {
                    ::core::result::Result::Ok(size) => {
                        max_size += size;
                    }
                    ::core::result::Result::Err(err) => {
                        break 'msg (::core::result::Result::<usize, _>::Err(err));
                    }
                }
                match ::micropb::const_map!(
                    ::core::result::Result::Ok(8usize), | size | size + 1usize
                ) {
                    ::core::result::Result::Ok(size) => {
                        max_size += size;
                    }
                    ::core::result::Result::Err(err) => {
                        break 'msg (::core::result::Result::<usize, _>::Err(err));
                    }
                }
                ::core::result::Result::Ok(max_size)
            };
            fn encode<IMPL_MICROPB_WRITE: ::micropb::PbWrite>(
                &self,
                encoder: &mut ::micropb::PbEncoder<IMPL_MICROPB_WRITE>,
            ) -> Result<(), IMPL_MICROPB_WRITE::Error> {
                use ::micropb::{PbMap, FieldEncode};
                {
                    let val_ref = &self.r#week;
                    if *val_ref != 0 {
                        encoder.encode_varint32(8u32)?;
                        encoder.encode_varint32(*val_ref as _)?;
                    }
                }
                {
                    let val_ref = &self.r#tow_s;
                    if *val_ref != 0.0 {
                        encoder.encode_varint32(17u32)?;
                        encoder.encode_double(*val_ref)?;
                    }
                }
                Ok(())
            }
            fn compute_size(&self) -> usize {
                use ::micropb::{PbMap, FieldEncode};
                let mut size = 0;
                {
                    let val_ref = &self.r#week;
                    if *val_ref != 0 {
                        size += 1usize + ::micropb::size::sizeof_varint32(*val_ref as _);
                    }
                }
                {
                    let val_ref = &self.r#tow_s;
                    if *val_ref != 0.0 {
                        size += 1usize + 8;
                    }
                }
                size
            }
        }
        /// A generic 3-vector. What the components mean depends on the field that
        /// carries it.
        #[derive(Debug, Default, PartialEq, Clone, Copy)]
        pub struct Vec3 {
            pub r#x: f64,
            pub r#y: f64,
            pub r#z: f64,
        }
        impl Vec3 {
            /// Return a reference to `x`
            #[inline]
            pub fn r#x(&self) -> &f64 {
                &self.r#x
            }
            /// Return a mutable reference to `x`
            #[inline]
            pub fn mut_x(&mut self) -> &mut f64 {
                &mut self.r#x
            }
            /// Set the value of `x`
            #[inline]
            pub fn set_x(&mut self, value: f64) -> &mut Self {
                self.r#x = value.into();
                self
            }
            /// Builder method that sets the value of `x`. Useful for initializing the message.
            #[inline]
            pub fn init_x(mut self, value: f64) -> Self {
                self.r#x = value.into();
                self
            }
            /// Return a reference to `y`
            #[inline]
            pub fn r#y(&self) -> &f64 {
                &self.r#y
            }
            /// Return a mutable reference to `y`
            #[inline]
            pub fn mut_y(&mut self) -> &mut f64 {
                &mut self.r#y
            }
            /// Set the value of `y`
            #[inline]
            pub fn set_y(&mut self, value: f64) -> &mut Self {
                self.r#y = value.into();
                self
            }
            /// Builder method that sets the value of `y`. Useful for initializing the message.
            #[inline]
            pub fn init_y(mut self, value: f64) -> Self {
                self.r#y = value.into();
                self
            }
            /// Return a reference to `z`
            #[inline]
            pub fn r#z(&self) -> &f64 {
                &self.r#z
            }
            /// Return a mutable reference to `z`
            #[inline]
            pub fn mut_z(&mut self) -> &mut f64 {
                &mut self.r#z
            }
            /// Set the value of `z`
            #[inline]
            pub fn set_z(&mut self, value: f64) -> &mut Self {
                self.r#z = value.into();
                self
            }
            /// Builder method that sets the value of `z`. Useful for initializing the message.
            #[inline]
            pub fn init_z(mut self, value: f64) -> Self {
                self.r#z = value.into();
                self
            }
        }
        impl ::micropb::MessageDecode for Vec3 {
            fn decode<IMPL_MICROPB_READ: ::micropb::PbRead>(
                &mut self,
                decoder: &mut ::micropb::PbDecoder<IMPL_MICROPB_READ>,
                len: usize,
            ) -> Result<(), ::micropb::DecodeError<IMPL_MICROPB_READ::Error>> {
                use ::micropb::{PbBytes, PbString, PbVec, PbMap, FieldDecode};
                let before = decoder.bytes_read();
                while decoder.bytes_read() - before < len {
                    let tag = decoder.decode_tag()?;
                    match tag.field_num() {
                        0 => return Err(::micropb::DecodeError::ZeroField),
                        1u32 => {
                            let mut_ref = &mut self.r#x;
                            {
                                let val = decoder.decode_double()?;
                                let val_ref = &val;
                                if *val_ref != 0.0 {
                                    *mut_ref = val as _;
                                }
                            };
                        }
                        2u32 => {
                            let mut_ref = &mut self.r#y;
                            {
                                let val = decoder.decode_double()?;
                                let val_ref = &val;
                                if *val_ref != 0.0 {
                                    *mut_ref = val as _;
                                }
                            };
                        }
                        3u32 => {
                            let mut_ref = &mut self.r#z;
                            {
                                let val = decoder.decode_double()?;
                                let val_ref = &val;
                                if *val_ref != 0.0 {
                                    *mut_ref = val as _;
                                }
                            };
                        }
                        _ => {
                            decoder.skip_wire_value(tag.wire_type())?;
                        }
                    }
                }
                Ok(())
            }
        }
        impl ::micropb::MessageEncode for Vec3 {
            const MAX_SIZE: ::core::result::Result<usize, &'static str> = 'msg: {
                let mut max_size = 0;
                match ::micropb::const_map!(
                    ::core::result::Result::Ok(8usize), | size | size + 1usize
                ) {
                    ::core::result::Result::Ok(size) => {
                        max_size += size;
                    }
                    ::core::result::Result::Err(err) => {
                        break 'msg (::core::result::Result::<usize, _>::Err(err));
                    }
                }
                match ::micropb::const_map!(
                    ::core::result::Result::Ok(8usize), | size | size + 1usize
                ) {
                    ::core::result::Result::Ok(size) => {
                        max_size += size;
                    }
                    ::core::result::Result::Err(err) => {
                        break 'msg (::core::result::Result::<usize, _>::Err(err));
                    }
                }
                match ::micropb::const_map!(
                    ::core::result::Result::Ok(8usize), | size | size + 1usize
                ) {
                    ::core::result::Result::Ok(size) => {
                        max_size += size;
                    }
                    ::core::result::Result::Err(err) => {
                        break 'msg (::core::result::Result::<usize, _>::Err(err));
                    }
                }
                ::core::result::Result::Ok(max_size)
            };
            fn encode<IMPL_MICROPB_WRITE: ::micropb::PbWrite>(
                &self,
                encoder: &mut ::micropb::PbEncoder<IMPL_MICROPB_WRITE>,
            ) -> Result<(), IMPL_MICROPB_WRITE::Error> {
                use ::micropb::{PbMap, FieldEncode};
                {
                    let val_ref = &self.r#x;
                    if *val_ref != 0.0 {
                        encoder.encode_varint32(9u32)?;
                        encoder.encode_double(*val_ref)?;
                    }
                }
                {
                    let val_ref = &self.r#y;
                    if *val_ref != 0.0 {
                        encoder.encode_varint32(17u32)?;
                        encoder.encode_double(*val_ref)?;
                    }
                }
                {
                    let val_ref = &self.r#z;
                    if *val_ref != 0.0 {
                        encoder.encode_varint32(25u32)?;
                        encoder.encode_double(*val_ref)?;
                    }
                }
                Ok(())
            }
            fn compute_size(&self) -> usize {
                use ::micropb::{PbMap, FieldEncode};
                let mut size = 0;
                {
                    let val_ref = &self.r#x;
                    if *val_ref != 0.0 {
                        size += 1usize + 8;
                    }
                }
                {
                    let val_ref = &self.r#y;
                    if *val_ref != 0.0 {
                        size += 1usize + 8;
                    }
                }
                {
                    let val_ref = &self.r#z;
                    if *val_ref != 0.0 {
                        size += 1usize + 8;
                    }
                }
                size
            }
        }
        /// Geodetic position on the WGS-84 ellipsoid.
        #[derive(Debug, Default, PartialEq, Clone, Copy)]
        pub struct Lla {
            /// Geodetic latitude, radians, positive north.
            pub r#lat_rad: f64,
            /// Longitude, radians, positive east.
            pub r#lon_rad: f64,
            /// Height above the WGS-84 ELLIPSOID, metres. Not orthometric height —
            /// a receiver reporting height above mean sea level must have the geoid
            /// undulation added back before it reaches this field.
            pub r#height_m: f64,
        }
        impl Lla {
            /// Return a reference to `lat_rad`
            #[inline]
            pub fn r#lat_rad(&self) -> &f64 {
                &self.r#lat_rad
            }
            /// Return a mutable reference to `lat_rad`
            #[inline]
            pub fn mut_lat_rad(&mut self) -> &mut f64 {
                &mut self.r#lat_rad
            }
            /// Set the value of `lat_rad`
            #[inline]
            pub fn set_lat_rad(&mut self, value: f64) -> &mut Self {
                self.r#lat_rad = value.into();
                self
            }
            /// Builder method that sets the value of `lat_rad`. Useful for initializing the message.
            #[inline]
            pub fn init_lat_rad(mut self, value: f64) -> Self {
                self.r#lat_rad = value.into();
                self
            }
            /// Return a reference to `lon_rad`
            #[inline]
            pub fn r#lon_rad(&self) -> &f64 {
                &self.r#lon_rad
            }
            /// Return a mutable reference to `lon_rad`
            #[inline]
            pub fn mut_lon_rad(&mut self) -> &mut f64 {
                &mut self.r#lon_rad
            }
            /// Set the value of `lon_rad`
            #[inline]
            pub fn set_lon_rad(&mut self, value: f64) -> &mut Self {
                self.r#lon_rad = value.into();
                self
            }
            /// Builder method that sets the value of `lon_rad`. Useful for initializing the message.
            #[inline]
            pub fn init_lon_rad(mut self, value: f64) -> Self {
                self.r#lon_rad = value.into();
                self
            }
            /// Return a reference to `height_m`
            #[inline]
            pub fn r#height_m(&self) -> &f64 {
                &self.r#height_m
            }
            /// Return a mutable reference to `height_m`
            #[inline]
            pub fn mut_height_m(&mut self) -> &mut f64 {
                &mut self.r#height_m
            }
            /// Set the value of `height_m`
            #[inline]
            pub fn set_height_m(&mut self, value: f64) -> &mut Self {
                self.r#height_m = value.into();
                self
            }
            /// Builder method that sets the value of `height_m`. Useful for initializing the message.
            #[inline]
            pub fn init_height_m(mut self, value: f64) -> Self {
                self.r#height_m = value.into();
                self
            }
        }
        impl ::micropb::MessageDecode for Lla {
            fn decode<IMPL_MICROPB_READ: ::micropb::PbRead>(
                &mut self,
                decoder: &mut ::micropb::PbDecoder<IMPL_MICROPB_READ>,
                len: usize,
            ) -> Result<(), ::micropb::DecodeError<IMPL_MICROPB_READ::Error>> {
                use ::micropb::{PbBytes, PbString, PbVec, PbMap, FieldDecode};
                let before = decoder.bytes_read();
                while decoder.bytes_read() - before < len {
                    let tag = decoder.decode_tag()?;
                    match tag.field_num() {
                        0 => return Err(::micropb::DecodeError::ZeroField),
                        1u32 => {
                            let mut_ref = &mut self.r#lat_rad;
                            {
                                let val = decoder.decode_double()?;
                                let val_ref = &val;
                                if *val_ref != 0.0 {
                                    *mut_ref = val as _;
                                }
                            };
                        }
                        2u32 => {
                            let mut_ref = &mut self.r#lon_rad;
                            {
                                let val = decoder.decode_double()?;
                                let val_ref = &val;
                                if *val_ref != 0.0 {
                                    *mut_ref = val as _;
                                }
                            };
                        }
                        3u32 => {
                            let mut_ref = &mut self.r#height_m;
                            {
                                let val = decoder.decode_double()?;
                                let val_ref = &val;
                                if *val_ref != 0.0 {
                                    *mut_ref = val as _;
                                }
                            };
                        }
                        _ => {
                            decoder.skip_wire_value(tag.wire_type())?;
                        }
                    }
                }
                Ok(())
            }
        }
        impl ::micropb::MessageEncode for Lla {
            const MAX_SIZE: ::core::result::Result<usize, &'static str> = 'msg: {
                let mut max_size = 0;
                match ::micropb::const_map!(
                    ::core::result::Result::Ok(8usize), | size | size + 1usize
                ) {
                    ::core::result::Result::Ok(size) => {
                        max_size += size;
                    }
                    ::core::result::Result::Err(err) => {
                        break 'msg (::core::result::Result::<usize, _>::Err(err));
                    }
                }
                match ::micropb::const_map!(
                    ::core::result::Result::Ok(8usize), | size | size + 1usize
                ) {
                    ::core::result::Result::Ok(size) => {
                        max_size += size;
                    }
                    ::core::result::Result::Err(err) => {
                        break 'msg (::core::result::Result::<usize, _>::Err(err));
                    }
                }
                match ::micropb::const_map!(
                    ::core::result::Result::Ok(8usize), | size | size + 1usize
                ) {
                    ::core::result::Result::Ok(size) => {
                        max_size += size;
                    }
                    ::core::result::Result::Err(err) => {
                        break 'msg (::core::result::Result::<usize, _>::Err(err));
                    }
                }
                ::core::result::Result::Ok(max_size)
            };
            fn encode<IMPL_MICROPB_WRITE: ::micropb::PbWrite>(
                &self,
                encoder: &mut ::micropb::PbEncoder<IMPL_MICROPB_WRITE>,
            ) -> Result<(), IMPL_MICROPB_WRITE::Error> {
                use ::micropb::{PbMap, FieldEncode};
                {
                    let val_ref = &self.r#lat_rad;
                    if *val_ref != 0.0 {
                        encoder.encode_varint32(9u32)?;
                        encoder.encode_double(*val_ref)?;
                    }
                }
                {
                    let val_ref = &self.r#lon_rad;
                    if *val_ref != 0.0 {
                        encoder.encode_varint32(17u32)?;
                        encoder.encode_double(*val_ref)?;
                    }
                }
                {
                    let val_ref = &self.r#height_m;
                    if *val_ref != 0.0 {
                        encoder.encode_varint32(25u32)?;
                        encoder.encode_double(*val_ref)?;
                    }
                }
                Ok(())
            }
            fn compute_size(&self) -> usize {
                use ::micropb::{PbMap, FieldEncode};
                let mut size = 0;
                {
                    let val_ref = &self.r#lat_rad;
                    if *val_ref != 0.0 {
                        size += 1usize + 8;
                    }
                }
                {
                    let val_ref = &self.r#lon_rad;
                    if *val_ref != 0.0 {
                        size += 1usize + 8;
                    }
                }
                {
                    let val_ref = &self.r#height_m;
                    if *val_ref != 0.0 {
                        size += 1usize + 8;
                    }
                }
                size
            }
        }
        /// A local tangent-plane displacement, north-east-down, metres.
        #[derive(Debug, Default, PartialEq, Clone, Copy)]
        pub struct Ned {
            pub r#north_m: f64,
            pub r#east_m: f64,
            pub r#down_m: f64,
        }
        impl Ned {
            /// Return a reference to `north_m`
            #[inline]
            pub fn r#north_m(&self) -> &f64 {
                &self.r#north_m
            }
            /// Return a mutable reference to `north_m`
            #[inline]
            pub fn mut_north_m(&mut self) -> &mut f64 {
                &mut self.r#north_m
            }
            /// Set the value of `north_m`
            #[inline]
            pub fn set_north_m(&mut self, value: f64) -> &mut Self {
                self.r#north_m = value.into();
                self
            }
            /// Builder method that sets the value of `north_m`. Useful for initializing the message.
            #[inline]
            pub fn init_north_m(mut self, value: f64) -> Self {
                self.r#north_m = value.into();
                self
            }
            /// Return a reference to `east_m`
            #[inline]
            pub fn r#east_m(&self) -> &f64 {
                &self.r#east_m
            }
            /// Return a mutable reference to `east_m`
            #[inline]
            pub fn mut_east_m(&mut self) -> &mut f64 {
                &mut self.r#east_m
            }
            /// Set the value of `east_m`
            #[inline]
            pub fn set_east_m(&mut self, value: f64) -> &mut Self {
                self.r#east_m = value.into();
                self
            }
            /// Builder method that sets the value of `east_m`. Useful for initializing the message.
            #[inline]
            pub fn init_east_m(mut self, value: f64) -> Self {
                self.r#east_m = value.into();
                self
            }
            /// Return a reference to `down_m`
            #[inline]
            pub fn r#down_m(&self) -> &f64 {
                &self.r#down_m
            }
            /// Return a mutable reference to `down_m`
            #[inline]
            pub fn mut_down_m(&mut self) -> &mut f64 {
                &mut self.r#down_m
            }
            /// Set the value of `down_m`
            #[inline]
            pub fn set_down_m(&mut self, value: f64) -> &mut Self {
                self.r#down_m = value.into();
                self
            }
            /// Builder method that sets the value of `down_m`. Useful for initializing the message.
            #[inline]
            pub fn init_down_m(mut self, value: f64) -> Self {
                self.r#down_m = value.into();
                self
            }
        }
        impl ::micropb::MessageDecode for Ned {
            fn decode<IMPL_MICROPB_READ: ::micropb::PbRead>(
                &mut self,
                decoder: &mut ::micropb::PbDecoder<IMPL_MICROPB_READ>,
                len: usize,
            ) -> Result<(), ::micropb::DecodeError<IMPL_MICROPB_READ::Error>> {
                use ::micropb::{PbBytes, PbString, PbVec, PbMap, FieldDecode};
                let before = decoder.bytes_read();
                while decoder.bytes_read() - before < len {
                    let tag = decoder.decode_tag()?;
                    match tag.field_num() {
                        0 => return Err(::micropb::DecodeError::ZeroField),
                        1u32 => {
                            let mut_ref = &mut self.r#north_m;
                            {
                                let val = decoder.decode_double()?;
                                let val_ref = &val;
                                if *val_ref != 0.0 {
                                    *mut_ref = val as _;
                                }
                            };
                        }
                        2u32 => {
                            let mut_ref = &mut self.r#east_m;
                            {
                                let val = decoder.decode_double()?;
                                let val_ref = &val;
                                if *val_ref != 0.0 {
                                    *mut_ref = val as _;
                                }
                            };
                        }
                        3u32 => {
                            let mut_ref = &mut self.r#down_m;
                            {
                                let val = decoder.decode_double()?;
                                let val_ref = &val;
                                if *val_ref != 0.0 {
                                    *mut_ref = val as _;
                                }
                            };
                        }
                        _ => {
                            decoder.skip_wire_value(tag.wire_type())?;
                        }
                    }
                }
                Ok(())
            }
        }
        impl ::micropb::MessageEncode for Ned {
            const MAX_SIZE: ::core::result::Result<usize, &'static str> = 'msg: {
                let mut max_size = 0;
                match ::micropb::const_map!(
                    ::core::result::Result::Ok(8usize), | size | size + 1usize
                ) {
                    ::core::result::Result::Ok(size) => {
                        max_size += size;
                    }
                    ::core::result::Result::Err(err) => {
                        break 'msg (::core::result::Result::<usize, _>::Err(err));
                    }
                }
                match ::micropb::const_map!(
                    ::core::result::Result::Ok(8usize), | size | size + 1usize
                ) {
                    ::core::result::Result::Ok(size) => {
                        max_size += size;
                    }
                    ::core::result::Result::Err(err) => {
                        break 'msg (::core::result::Result::<usize, _>::Err(err));
                    }
                }
                match ::micropb::const_map!(
                    ::core::result::Result::Ok(8usize), | size | size + 1usize
                ) {
                    ::core::result::Result::Ok(size) => {
                        max_size += size;
                    }
                    ::core::result::Result::Err(err) => {
                        break 'msg (::core::result::Result::<usize, _>::Err(err));
                    }
                }
                ::core::result::Result::Ok(max_size)
            };
            fn encode<IMPL_MICROPB_WRITE: ::micropb::PbWrite>(
                &self,
                encoder: &mut ::micropb::PbEncoder<IMPL_MICROPB_WRITE>,
            ) -> Result<(), IMPL_MICROPB_WRITE::Error> {
                use ::micropb::{PbMap, FieldEncode};
                {
                    let val_ref = &self.r#north_m;
                    if *val_ref != 0.0 {
                        encoder.encode_varint32(9u32)?;
                        encoder.encode_double(*val_ref)?;
                    }
                }
                {
                    let val_ref = &self.r#east_m;
                    if *val_ref != 0.0 {
                        encoder.encode_varint32(17u32)?;
                        encoder.encode_double(*val_ref)?;
                    }
                }
                {
                    let val_ref = &self.r#down_m;
                    if *val_ref != 0.0 {
                        encoder.encode_varint32(25u32)?;
                        encoder.encode_double(*val_ref)?;
                    }
                }
                Ok(())
            }
            fn compute_size(&self) -> usize {
                use ::micropb::{PbMap, FieldEncode};
                let mut size = 0;
                {
                    let val_ref = &self.r#north_m;
                    if *val_ref != 0.0 {
                        size += 1usize + 8;
                    }
                }
                {
                    let val_ref = &self.r#east_m;
                    if *val_ref != 0.0 {
                        size += 1usize + 8;
                    }
                }
                {
                    let val_ref = &self.r#down_m;
                    if *val_ref != 0.0 {
                        size += 1usize + 8;
                    }
                }
                size
            }
        }
        /// Attitude as a Hamilton quaternion q_nb, rotating body vectors into the
        /// navigation frame. Scalar first, normally unit length.
        #[derive(Debug, Default, PartialEq, Clone, Copy)]
        pub struct Quaternion {
            pub r#w: f64,
            pub r#x: f64,
            pub r#y: f64,
            pub r#z: f64,
        }
        impl Quaternion {
            /// Return a reference to `w`
            #[inline]
            pub fn r#w(&self) -> &f64 {
                &self.r#w
            }
            /// Return a mutable reference to `w`
            #[inline]
            pub fn mut_w(&mut self) -> &mut f64 {
                &mut self.r#w
            }
            /// Set the value of `w`
            #[inline]
            pub fn set_w(&mut self, value: f64) -> &mut Self {
                self.r#w = value.into();
                self
            }
            /// Builder method that sets the value of `w`. Useful for initializing the message.
            #[inline]
            pub fn init_w(mut self, value: f64) -> Self {
                self.r#w = value.into();
                self
            }
            /// Return a reference to `x`
            #[inline]
            pub fn r#x(&self) -> &f64 {
                &self.r#x
            }
            /// Return a mutable reference to `x`
            #[inline]
            pub fn mut_x(&mut self) -> &mut f64 {
                &mut self.r#x
            }
            /// Set the value of `x`
            #[inline]
            pub fn set_x(&mut self, value: f64) -> &mut Self {
                self.r#x = value.into();
                self
            }
            /// Builder method that sets the value of `x`. Useful for initializing the message.
            #[inline]
            pub fn init_x(mut self, value: f64) -> Self {
                self.r#x = value.into();
                self
            }
            /// Return a reference to `y`
            #[inline]
            pub fn r#y(&self) -> &f64 {
                &self.r#y
            }
            /// Return a mutable reference to `y`
            #[inline]
            pub fn mut_y(&mut self) -> &mut f64 {
                &mut self.r#y
            }
            /// Set the value of `y`
            #[inline]
            pub fn set_y(&mut self, value: f64) -> &mut Self {
                self.r#y = value.into();
                self
            }
            /// Builder method that sets the value of `y`. Useful for initializing the message.
            #[inline]
            pub fn init_y(mut self, value: f64) -> Self {
                self.r#y = value.into();
                self
            }
            /// Return a reference to `z`
            #[inline]
            pub fn r#z(&self) -> &f64 {
                &self.r#z
            }
            /// Return a mutable reference to `z`
            #[inline]
            pub fn mut_z(&mut self) -> &mut f64 {
                &mut self.r#z
            }
            /// Set the value of `z`
            #[inline]
            pub fn set_z(&mut self, value: f64) -> &mut Self {
                self.r#z = value.into();
                self
            }
            /// Builder method that sets the value of `z`. Useful for initializing the message.
            #[inline]
            pub fn init_z(mut self, value: f64) -> Self {
                self.r#z = value.into();
                self
            }
        }
        impl ::micropb::MessageDecode for Quaternion {
            fn decode<IMPL_MICROPB_READ: ::micropb::PbRead>(
                &mut self,
                decoder: &mut ::micropb::PbDecoder<IMPL_MICROPB_READ>,
                len: usize,
            ) -> Result<(), ::micropb::DecodeError<IMPL_MICROPB_READ::Error>> {
                use ::micropb::{PbBytes, PbString, PbVec, PbMap, FieldDecode};
                let before = decoder.bytes_read();
                while decoder.bytes_read() - before < len {
                    let tag = decoder.decode_tag()?;
                    match tag.field_num() {
                        0 => return Err(::micropb::DecodeError::ZeroField),
                        1u32 => {
                            let mut_ref = &mut self.r#w;
                            {
                                let val = decoder.decode_double()?;
                                let val_ref = &val;
                                if *val_ref != 0.0 {
                                    *mut_ref = val as _;
                                }
                            };
                        }
                        2u32 => {
                            let mut_ref = &mut self.r#x;
                            {
                                let val = decoder.decode_double()?;
                                let val_ref = &val;
                                if *val_ref != 0.0 {
                                    *mut_ref = val as _;
                                }
                            };
                        }
                        3u32 => {
                            let mut_ref = &mut self.r#y;
                            {
                                let val = decoder.decode_double()?;
                                let val_ref = &val;
                                if *val_ref != 0.0 {
                                    *mut_ref = val as _;
                                }
                            };
                        }
                        4u32 => {
                            let mut_ref = &mut self.r#z;
                            {
                                let val = decoder.decode_double()?;
                                let val_ref = &val;
                                if *val_ref != 0.0 {
                                    *mut_ref = val as _;
                                }
                            };
                        }
                        _ => {
                            decoder.skip_wire_value(tag.wire_type())?;
                        }
                    }
                }
                Ok(())
            }
        }
        impl ::micropb::MessageEncode for Quaternion {
            const MAX_SIZE: ::core::result::Result<usize, &'static str> = 'msg: {
                let mut max_size = 0;
                match ::micropb::const_map!(
                    ::core::result::Result::Ok(8usize), | size | size + 1usize
                ) {
                    ::core::result::Result::Ok(size) => {
                        max_size += size;
                    }
                    ::core::result::Result::Err(err) => {
                        break 'msg (::core::result::Result::<usize, _>::Err(err));
                    }
                }
                match ::micropb::const_map!(
                    ::core::result::Result::Ok(8usize), | size | size + 1usize
                ) {
                    ::core::result::Result::Ok(size) => {
                        max_size += size;
                    }
                    ::core::result::Result::Err(err) => {
                        break 'msg (::core::result::Result::<usize, _>::Err(err));
                    }
                }
                match ::micropb::const_map!(
                    ::core::result::Result::Ok(8usize), | size | size + 1usize
                ) {
                    ::core::result::Result::Ok(size) => {
                        max_size += size;
                    }
                    ::core::result::Result::Err(err) => {
                        break 'msg (::core::result::Result::<usize, _>::Err(err));
                    }
                }
                match ::micropb::const_map!(
                    ::core::result::Result::Ok(8usize), | size | size + 1usize
                ) {
                    ::core::result::Result::Ok(size) => {
                        max_size += size;
                    }
                    ::core::result::Result::Err(err) => {
                        break 'msg (::core::result::Result::<usize, _>::Err(err));
                    }
                }
                ::core::result::Result::Ok(max_size)
            };
            fn encode<IMPL_MICROPB_WRITE: ::micropb::PbWrite>(
                &self,
                encoder: &mut ::micropb::PbEncoder<IMPL_MICROPB_WRITE>,
            ) -> Result<(), IMPL_MICROPB_WRITE::Error> {
                use ::micropb::{PbMap, FieldEncode};
                {
                    let val_ref = &self.r#w;
                    if *val_ref != 0.0 {
                        encoder.encode_varint32(9u32)?;
                        encoder.encode_double(*val_ref)?;
                    }
                }
                {
                    let val_ref = &self.r#x;
                    if *val_ref != 0.0 {
                        encoder.encode_varint32(17u32)?;
                        encoder.encode_double(*val_ref)?;
                    }
                }
                {
                    let val_ref = &self.r#y;
                    if *val_ref != 0.0 {
                        encoder.encode_varint32(25u32)?;
                        encoder.encode_double(*val_ref)?;
                    }
                }
                {
                    let val_ref = &self.r#z;
                    if *val_ref != 0.0 {
                        encoder.encode_varint32(33u32)?;
                        encoder.encode_double(*val_ref)?;
                    }
                }
                Ok(())
            }
            fn compute_size(&self) -> usize {
                use ::micropb::{PbMap, FieldEncode};
                let mut size = 0;
                {
                    let val_ref = &self.r#w;
                    if *val_ref != 0.0 {
                        size += 1usize + 8;
                    }
                }
                {
                    let val_ref = &self.r#x;
                    if *val_ref != 0.0 {
                        size += 1usize + 8;
                    }
                }
                {
                    let val_ref = &self.r#y;
                    if *val_ref != 0.0 {
                        size += 1usize + 8;
                    }
                }
                {
                    let val_ref = &self.r#z;
                    if *val_ref != 0.0 {
                        size += 1usize + 8;
                    }
                }
                size
            }
        }
        /// Roll, pitch, yaw in radians, Z-Y-X aerospace sequence. Output only —
        /// the filter never uses Euler angles internally.
        #[derive(Debug, Default, PartialEq, Clone, Copy)]
        pub struct Euler {
            pub r#roll_rad: f64,
            pub r#pitch_rad: f64,
            pub r#yaw_rad: f64,
        }
        impl Euler {
            /// Return a reference to `roll_rad`
            #[inline]
            pub fn r#roll_rad(&self) -> &f64 {
                &self.r#roll_rad
            }
            /// Return a mutable reference to `roll_rad`
            #[inline]
            pub fn mut_roll_rad(&mut self) -> &mut f64 {
                &mut self.r#roll_rad
            }
            /// Set the value of `roll_rad`
            #[inline]
            pub fn set_roll_rad(&mut self, value: f64) -> &mut Self {
                self.r#roll_rad = value.into();
                self
            }
            /// Builder method that sets the value of `roll_rad`. Useful for initializing the message.
            #[inline]
            pub fn init_roll_rad(mut self, value: f64) -> Self {
                self.r#roll_rad = value.into();
                self
            }
            /// Return a reference to `pitch_rad`
            #[inline]
            pub fn r#pitch_rad(&self) -> &f64 {
                &self.r#pitch_rad
            }
            /// Return a mutable reference to `pitch_rad`
            #[inline]
            pub fn mut_pitch_rad(&mut self) -> &mut f64 {
                &mut self.r#pitch_rad
            }
            /// Set the value of `pitch_rad`
            #[inline]
            pub fn set_pitch_rad(&mut self, value: f64) -> &mut Self {
                self.r#pitch_rad = value.into();
                self
            }
            /// Builder method that sets the value of `pitch_rad`. Useful for initializing the message.
            #[inline]
            pub fn init_pitch_rad(mut self, value: f64) -> Self {
                self.r#pitch_rad = value.into();
                self
            }
            /// Return a reference to `yaw_rad`
            #[inline]
            pub fn r#yaw_rad(&self) -> &f64 {
                &self.r#yaw_rad
            }
            /// Return a mutable reference to `yaw_rad`
            #[inline]
            pub fn mut_yaw_rad(&mut self) -> &mut f64 {
                &mut self.r#yaw_rad
            }
            /// Set the value of `yaw_rad`
            #[inline]
            pub fn set_yaw_rad(&mut self, value: f64) -> &mut Self {
                self.r#yaw_rad = value.into();
                self
            }
            /// Builder method that sets the value of `yaw_rad`. Useful for initializing the message.
            #[inline]
            pub fn init_yaw_rad(mut self, value: f64) -> Self {
                self.r#yaw_rad = value.into();
                self
            }
        }
        impl ::micropb::MessageDecode for Euler {
            fn decode<IMPL_MICROPB_READ: ::micropb::PbRead>(
                &mut self,
                decoder: &mut ::micropb::PbDecoder<IMPL_MICROPB_READ>,
                len: usize,
            ) -> Result<(), ::micropb::DecodeError<IMPL_MICROPB_READ::Error>> {
                use ::micropb::{PbBytes, PbString, PbVec, PbMap, FieldDecode};
                let before = decoder.bytes_read();
                while decoder.bytes_read() - before < len {
                    let tag = decoder.decode_tag()?;
                    match tag.field_num() {
                        0 => return Err(::micropb::DecodeError::ZeroField),
                        1u32 => {
                            let mut_ref = &mut self.r#roll_rad;
                            {
                                let val = decoder.decode_double()?;
                                let val_ref = &val;
                                if *val_ref != 0.0 {
                                    *mut_ref = val as _;
                                }
                            };
                        }
                        2u32 => {
                            let mut_ref = &mut self.r#pitch_rad;
                            {
                                let val = decoder.decode_double()?;
                                let val_ref = &val;
                                if *val_ref != 0.0 {
                                    *mut_ref = val as _;
                                }
                            };
                        }
                        3u32 => {
                            let mut_ref = &mut self.r#yaw_rad;
                            {
                                let val = decoder.decode_double()?;
                                let val_ref = &val;
                                if *val_ref != 0.0 {
                                    *mut_ref = val as _;
                                }
                            };
                        }
                        _ => {
                            decoder.skip_wire_value(tag.wire_type())?;
                        }
                    }
                }
                Ok(())
            }
        }
        impl ::micropb::MessageEncode for Euler {
            const MAX_SIZE: ::core::result::Result<usize, &'static str> = 'msg: {
                let mut max_size = 0;
                match ::micropb::const_map!(
                    ::core::result::Result::Ok(8usize), | size | size + 1usize
                ) {
                    ::core::result::Result::Ok(size) => {
                        max_size += size;
                    }
                    ::core::result::Result::Err(err) => {
                        break 'msg (::core::result::Result::<usize, _>::Err(err));
                    }
                }
                match ::micropb::const_map!(
                    ::core::result::Result::Ok(8usize), | size | size + 1usize
                ) {
                    ::core::result::Result::Ok(size) => {
                        max_size += size;
                    }
                    ::core::result::Result::Err(err) => {
                        break 'msg (::core::result::Result::<usize, _>::Err(err));
                    }
                }
                match ::micropb::const_map!(
                    ::core::result::Result::Ok(8usize), | size | size + 1usize
                ) {
                    ::core::result::Result::Ok(size) => {
                        max_size += size;
                    }
                    ::core::result::Result::Err(err) => {
                        break 'msg (::core::result::Result::<usize, _>::Err(err));
                    }
                }
                ::core::result::Result::Ok(max_size)
            };
            fn encode<IMPL_MICROPB_WRITE: ::micropb::PbWrite>(
                &self,
                encoder: &mut ::micropb::PbEncoder<IMPL_MICROPB_WRITE>,
            ) -> Result<(), IMPL_MICROPB_WRITE::Error> {
                use ::micropb::{PbMap, FieldEncode};
                {
                    let val_ref = &self.r#roll_rad;
                    if *val_ref != 0.0 {
                        encoder.encode_varint32(9u32)?;
                        encoder.encode_double(*val_ref)?;
                    }
                }
                {
                    let val_ref = &self.r#pitch_rad;
                    if *val_ref != 0.0 {
                        encoder.encode_varint32(17u32)?;
                        encoder.encode_double(*val_ref)?;
                    }
                }
                {
                    let val_ref = &self.r#yaw_rad;
                    if *val_ref != 0.0 {
                        encoder.encode_varint32(25u32)?;
                        encoder.encode_double(*val_ref)?;
                    }
                }
                Ok(())
            }
            fn compute_size(&self) -> usize {
                use ::micropb::{PbMap, FieldEncode};
                let mut size = 0;
                {
                    let val_ref = &self.r#roll_rad;
                    if *val_ref != 0.0 {
                        size += 1usize + 8;
                    }
                }
                {
                    let val_ref = &self.r#pitch_rad;
                    if *val_ref != 0.0 {
                        size += 1usize + 8;
                    }
                }
                {
                    let val_ref = &self.r#yaw_rad;
                    if *val_ref != 0.0 {
                        size += 1usize + 8;
                    }
                }
                size
            }
        }
        /// One inertial measurement, in INCREMENTAL form.
        ///
        /// `dtheta` and `dvel` are the integrals of angular rate and specific force over
        /// `dt`, which is what a coning/sculling-corrected IMU reports natively and what
        /// the two-sample mechanization consumes. Producers holding instantaneous rates
        /// must multiply by `dt` before filling these in.
        #[derive(Debug, Default, Clone, Copy)]
        pub struct ImuSample {
            /// Timestamp at the END of the integration interval.
            ///
            /// *Note:* The presence of this field is tracked separately in the `_has` field. It's recommended to access this field via the accessor rather than directly.
            pub r#time: GpsTime,
            /// Length of the integration interval, seconds. Must be > 0.
            pub r#dt_s: f64,
            /// Integrated angular increment about the body axes (forward-right-down),
            /// radians.
            ///
            /// *Note:* The presence of this field is tracked separately in the `_has` field. It's recommended to access this field via the accessor rather than directly.
            pub r#dtheta_rad: Vec3,
            /// Integrated specific-force increment along the body axes, m/s.
            ///
            /// Specific force, not acceleration: a stationary level unit reads +9.8 m/s²
            /// upward, so `dvel.z` is about -9.8 * dt.
            ///
            /// *Note:* The presence of this field is tracked separately in the `_has` field. It's recommended to access this field via the accessor rather than directly.
            pub r#dvel_mps: Vec3,
            /// Tracks presence of optional and message fields
            pub _has: ImuSample_::_Hazzer,
        }
        impl ::core::cmp::PartialEq for ImuSample {
            fn eq(&self, other: &Self) -> bool {
                let mut ret = true;
                ret &= (self.r#time() == other.r#time());
                ret &= (self.r#dt_s == other.r#dt_s);
                ret &= (self.r#dtheta_rad() == other.r#dtheta_rad());
                ret &= (self.r#dvel_mps() == other.r#dvel_mps());
                ret
            }
        }
        impl ImuSample {
            /// Return a reference to `time` as an `Option`
            #[inline]
            pub fn r#time(&self) -> ::core::option::Option<&GpsTime> {
                self._has.r#time().then_some(&self.r#time)
            }
            /// Set the value and presence of `time`
            #[inline]
            pub fn set_time(&mut self, value: GpsTime) -> &mut Self {
                self._has.set_time();
                self.r#time = value.into();
                self
            }
            /// Return a mutable reference to `time` as an `Option`
            #[inline]
            pub fn mut_time(&mut self) -> ::core::option::Option<&mut GpsTime> {
                self._has.r#time().then_some(&mut self.r#time)
            }
            /// Clear the presence of `time`
            #[inline]
            pub fn clear_time(&mut self) -> &mut Self {
                self._has.clear_time();
                self
            }
            /// Take the value of `time` and clear its presence
            #[inline]
            pub fn take_time(&mut self) -> ::core::option::Option<GpsTime> {
                let val = self
                    ._has
                    .r#time()
                    .then(|| ::core::mem::take(&mut self.r#time));
                self._has.clear_time();
                val
            }
            /// Builder method that sets the value of `time`. Useful for initializing the message.
            #[inline]
            pub fn init_time(mut self, value: GpsTime) -> Self {
                self.set_time(value);
                self
            }
            /// Return a reference to `dt_s`
            #[inline]
            pub fn r#dt_s(&self) -> &f64 {
                &self.r#dt_s
            }
            /// Return a mutable reference to `dt_s`
            #[inline]
            pub fn mut_dt_s(&mut self) -> &mut f64 {
                &mut self.r#dt_s
            }
            /// Set the value of `dt_s`
            #[inline]
            pub fn set_dt_s(&mut self, value: f64) -> &mut Self {
                self.r#dt_s = value.into();
                self
            }
            /// Builder method that sets the value of `dt_s`. Useful for initializing the message.
            #[inline]
            pub fn init_dt_s(mut self, value: f64) -> Self {
                self.r#dt_s = value.into();
                self
            }
            /// Return a reference to `dtheta_rad` as an `Option`
            #[inline]
            pub fn r#dtheta_rad(&self) -> ::core::option::Option<&Vec3> {
                self._has.r#dtheta_rad().then_some(&self.r#dtheta_rad)
            }
            /// Set the value and presence of `dtheta_rad`
            #[inline]
            pub fn set_dtheta_rad(&mut self, value: Vec3) -> &mut Self {
                self._has.set_dtheta_rad();
                self.r#dtheta_rad = value.into();
                self
            }
            /// Return a mutable reference to `dtheta_rad` as an `Option`
            #[inline]
            pub fn mut_dtheta_rad(&mut self) -> ::core::option::Option<&mut Vec3> {
                self._has.r#dtheta_rad().then_some(&mut self.r#dtheta_rad)
            }
            /// Clear the presence of `dtheta_rad`
            #[inline]
            pub fn clear_dtheta_rad(&mut self) -> &mut Self {
                self._has.clear_dtheta_rad();
                self
            }
            /// Take the value of `dtheta_rad` and clear its presence
            #[inline]
            pub fn take_dtheta_rad(&mut self) -> ::core::option::Option<Vec3> {
                let val = self
                    ._has
                    .r#dtheta_rad()
                    .then(|| ::core::mem::take(&mut self.r#dtheta_rad));
                self._has.clear_dtheta_rad();
                val
            }
            /// Builder method that sets the value of `dtheta_rad`. Useful for initializing the message.
            #[inline]
            pub fn init_dtheta_rad(mut self, value: Vec3) -> Self {
                self.set_dtheta_rad(value);
                self
            }
            /// Return a reference to `dvel_mps` as an `Option`
            #[inline]
            pub fn r#dvel_mps(&self) -> ::core::option::Option<&Vec3> {
                self._has.r#dvel_mps().then_some(&self.r#dvel_mps)
            }
            /// Set the value and presence of `dvel_mps`
            #[inline]
            pub fn set_dvel_mps(&mut self, value: Vec3) -> &mut Self {
                self._has.set_dvel_mps();
                self.r#dvel_mps = value.into();
                self
            }
            /// Return a mutable reference to `dvel_mps` as an `Option`
            #[inline]
            pub fn mut_dvel_mps(&mut self) -> ::core::option::Option<&mut Vec3> {
                self._has.r#dvel_mps().then_some(&mut self.r#dvel_mps)
            }
            /// Clear the presence of `dvel_mps`
            #[inline]
            pub fn clear_dvel_mps(&mut self) -> &mut Self {
                self._has.clear_dvel_mps();
                self
            }
            /// Take the value of `dvel_mps` and clear its presence
            #[inline]
            pub fn take_dvel_mps(&mut self) -> ::core::option::Option<Vec3> {
                let val = self
                    ._has
                    .r#dvel_mps()
                    .then(|| ::core::mem::take(&mut self.r#dvel_mps));
                self._has.clear_dvel_mps();
                val
            }
            /// Builder method that sets the value of `dvel_mps`. Useful for initializing the message.
            #[inline]
            pub fn init_dvel_mps(mut self, value: Vec3) -> Self {
                self.set_dvel_mps(value);
                self
            }
        }
        impl ::micropb::MessageDecode for ImuSample {
            fn decode<IMPL_MICROPB_READ: ::micropb::PbRead>(
                &mut self,
                decoder: &mut ::micropb::PbDecoder<IMPL_MICROPB_READ>,
                len: usize,
            ) -> Result<(), ::micropb::DecodeError<IMPL_MICROPB_READ::Error>> {
                use ::micropb::{PbBytes, PbString, PbVec, PbMap, FieldDecode};
                let before = decoder.bytes_read();
                while decoder.bytes_read() - before < len {
                    let tag = decoder.decode_tag()?;
                    match tag.field_num() {
                        0 => return Err(::micropb::DecodeError::ZeroField),
                        1u32 => {
                            let mut_ref = &mut self.r#time;
                            {
                                mut_ref.decode_len_delimited(decoder)?;
                            };
                            self._has.set_time();
                        }
                        2u32 => {
                            let mut_ref = &mut self.r#dt_s;
                            {
                                let val = decoder.decode_double()?;
                                let val_ref = &val;
                                if *val_ref != 0.0 {
                                    *mut_ref = val as _;
                                }
                            };
                        }
                        3u32 => {
                            let mut_ref = &mut self.r#dtheta_rad;
                            {
                                mut_ref.decode_len_delimited(decoder)?;
                            };
                            self._has.set_dtheta_rad();
                        }
                        4u32 => {
                            let mut_ref = &mut self.r#dvel_mps;
                            {
                                mut_ref.decode_len_delimited(decoder)?;
                            };
                            self._has.set_dvel_mps();
                        }
                        _ => {
                            decoder.skip_wire_value(tag.wire_type())?;
                        }
                    }
                }
                Ok(())
            }
        }
        impl ::micropb::MessageEncode for ImuSample {
            const MAX_SIZE: ::core::result::Result<usize, &'static str> = 'msg: {
                let mut max_size = 0;
                match ::micropb::const_map!(
                    ::micropb::const_map!(< GpsTime as ::micropb::MessageEncode >
                    ::MAX_SIZE, | size | ::micropb::size::sizeof_len_record(size)), |
                    size | size + 1usize
                ) {
                    ::core::result::Result::Ok(size) => {
                        max_size += size;
                    }
                    ::core::result::Result::Err(err) => {
                        break 'msg (::core::result::Result::<usize, _>::Err(err));
                    }
                }
                match ::micropb::const_map!(
                    ::core::result::Result::Ok(8usize), | size | size + 1usize
                ) {
                    ::core::result::Result::Ok(size) => {
                        max_size += size;
                    }
                    ::core::result::Result::Err(err) => {
                        break 'msg (::core::result::Result::<usize, _>::Err(err));
                    }
                }
                match ::micropb::const_map!(
                    ::micropb::const_map!(< Vec3 as ::micropb::MessageEncode >
                    ::MAX_SIZE, | size | ::micropb::size::sizeof_len_record(size)), |
                    size | size + 1usize
                ) {
                    ::core::result::Result::Ok(size) => {
                        max_size += size;
                    }
                    ::core::result::Result::Err(err) => {
                        break 'msg (::core::result::Result::<usize, _>::Err(err));
                    }
                }
                match ::micropb::const_map!(
                    ::micropb::const_map!(< Vec3 as ::micropb::MessageEncode >
                    ::MAX_SIZE, | size | ::micropb::size::sizeof_len_record(size)), |
                    size | size + 1usize
                ) {
                    ::core::result::Result::Ok(size) => {
                        max_size += size;
                    }
                    ::core::result::Result::Err(err) => {
                        break 'msg (::core::result::Result::<usize, _>::Err(err));
                    }
                }
                ::core::result::Result::Ok(max_size)
            };
            fn encode<IMPL_MICROPB_WRITE: ::micropb::PbWrite>(
                &self,
                encoder: &mut ::micropb::PbEncoder<IMPL_MICROPB_WRITE>,
            ) -> Result<(), IMPL_MICROPB_WRITE::Error> {
                use ::micropb::{PbMap, FieldEncode};
                {
                    if let ::core::option::Option::Some(val_ref) = self.r#time() {
                        encoder.encode_varint32(10u32)?;
                        val_ref.encode_len_delimited(encoder)?;
                    }
                }
                {
                    let val_ref = &self.r#dt_s;
                    if *val_ref != 0.0 {
                        encoder.encode_varint32(17u32)?;
                        encoder.encode_double(*val_ref)?;
                    }
                }
                {
                    if let ::core::option::Option::Some(val_ref) = self.r#dtheta_rad() {
                        encoder.encode_varint32(26u32)?;
                        val_ref.encode_len_delimited(encoder)?;
                    }
                }
                {
                    if let ::core::option::Option::Some(val_ref) = self.r#dvel_mps() {
                        encoder.encode_varint32(34u32)?;
                        val_ref.encode_len_delimited(encoder)?;
                    }
                }
                Ok(())
            }
            fn compute_size(&self) -> usize {
                use ::micropb::{PbMap, FieldEncode};
                let mut size = 0;
                {
                    if let ::core::option::Option::Some(val_ref) = self.r#time() {
                        size
                            += 1usize
                                + ::micropb::size::sizeof_len_record(
                                    val_ref.compute_size(),
                                );
                    }
                }
                {
                    let val_ref = &self.r#dt_s;
                    if *val_ref != 0.0 {
                        size += 1usize + 8;
                    }
                }
                {
                    if let ::core::option::Option::Some(val_ref) = self.r#dtheta_rad() {
                        size
                            += 1usize
                                + ::micropb::size::sizeof_len_record(
                                    val_ref.compute_size(),
                                );
                    }
                }
                {
                    if let ::core::option::Option::Some(val_ref) = self.r#dvel_mps() {
                        size
                            += 1usize
                                + ::micropb::size::sizeof_len_record(
                                    val_ref.compute_size(),
                                );
                    }
                }
                size
            }
        }
        /// Inner types for `ImuSample`
        pub mod ImuSample_ {
            /// Compact bitfield for tracking presence of optional and message fields
            #[derive(Debug, Default, PartialEq, Clone, Copy)]
            pub struct _Hazzer([u8; 1]);
            impl _Hazzer {
                /// New hazzer with all fields set to off
                #[inline]
                pub const fn _new() -> Self {
                    Self([0; 1])
                }
                /// Query presence of `time`
                #[inline]
                pub const fn r#time(&self) -> bool {
                    (self.0[0] & 1) != 0
                }
                /// Set presence of `time`
                #[inline]
                pub const fn set_time(&mut self) -> &mut Self {
                    let elem = &mut self.0[0];
                    *elem |= 1;
                    self
                }
                /// Clear presence of `time`
                #[inline]
                pub const fn clear_time(&mut self) -> &mut Self {
                    let elem = &mut self.0[0];
                    *elem &= !1;
                    self
                }
                /// Builder method that sets the presence of `time`. Useful for initializing the Hazzer.
                #[inline]
                pub const fn init_time(mut self) -> Self {
                    self.set_time();
                    self
                }
                /// Query presence of `dtheta_rad`
                #[inline]
                pub const fn r#dtheta_rad(&self) -> bool {
                    (self.0[0] & 2) != 0
                }
                /// Set presence of `dtheta_rad`
                #[inline]
                pub const fn set_dtheta_rad(&mut self) -> &mut Self {
                    let elem = &mut self.0[0];
                    *elem |= 2;
                    self
                }
                /// Clear presence of `dtheta_rad`
                #[inline]
                pub const fn clear_dtheta_rad(&mut self) -> &mut Self {
                    let elem = &mut self.0[0];
                    *elem &= !2;
                    self
                }
                /// Builder method that sets the presence of `dtheta_rad`. Useful for initializing the Hazzer.
                #[inline]
                pub const fn init_dtheta_rad(mut self) -> Self {
                    self.set_dtheta_rad();
                    self
                }
                /// Query presence of `dvel_mps`
                #[inline]
                pub const fn r#dvel_mps(&self) -> bool {
                    (self.0[0] & 4) != 0
                }
                /// Set presence of `dvel_mps`
                #[inline]
                pub const fn set_dvel_mps(&mut self) -> &mut Self {
                    let elem = &mut self.0[0];
                    *elem |= 4;
                    self
                }
                /// Clear presence of `dvel_mps`
                #[inline]
                pub const fn clear_dvel_mps(&mut self) -> &mut Self {
                    let elem = &mut self.0[0];
                    *elem &= !4;
                    self
                }
                /// Builder method that sets the presence of `dvel_mps`. Useful for initializing the Hazzer.
                #[inline]
                pub const fn init_dvel_mps(mut self) -> Self {
                    self.set_dvel_mps();
                    self
                }
            }
        }
        /// A GNSS position fix — the loosely-coupled measurement.
        #[derive(Debug, Default, Clone, Copy)]
        pub struct GnssFix {
            /// *Note:* The presence of this field is tracked separately in the `_has` field. It's recommended to access this field via the accessor rather than directly.
            pub r#time: GpsTime,
            /// Antenna phase-centre position.
            ///
            /// *Note:* The presence of this field is tracked separately in the `_has` field. It's recommended to access this field via the accessor rather than directly.
            pub r#position: Lla,
            /// One-sigma position uncertainty in the local NED frame, metres. Every
            /// component must be strictly positive; a zero makes the innovation
            /// covariance singular and the fix is rejected.
            ///
            /// *Note:* The presence of this field is tracked separately in the `_has` field. It's recommended to access this field via the accessor rather than directly.
            pub r#position_std_m: Vec3,
            /// Ground velocity in NED, m/s. Absent when the receiver does not report it —
            /// `optional` so that "absent" and "zero velocity" stay distinguishable.
            ///
            /// *Note:* The presence of this field is tracked separately in the `_has` field. It's recommended to access this field via the accessor rather than directly.
            pub r#velocity_mps: Ned,
            /// One-sigma velocity uncertainty in NED, m/s.
            ///
            /// *Note:* The presence of this field is tracked separately in the `_has` field. It's recommended to access this field via the accessor rather than directly.
            pub r#velocity_std_mps: Vec3,
            /// Tracks presence of optional and message fields
            pub _has: GnssFix_::_Hazzer,
        }
        impl ::core::cmp::PartialEq for GnssFix {
            fn eq(&self, other: &Self) -> bool {
                let mut ret = true;
                ret &= (self.r#time() == other.r#time());
                ret &= (self.r#position() == other.r#position());
                ret &= (self.r#position_std_m() == other.r#position_std_m());
                ret &= (self.r#velocity_mps() == other.r#velocity_mps());
                ret &= (self.r#velocity_std_mps() == other.r#velocity_std_mps());
                ret
            }
        }
        impl GnssFix {
            /// Return a reference to `time` as an `Option`
            #[inline]
            pub fn r#time(&self) -> ::core::option::Option<&GpsTime> {
                self._has.r#time().then_some(&self.r#time)
            }
            /// Set the value and presence of `time`
            #[inline]
            pub fn set_time(&mut self, value: GpsTime) -> &mut Self {
                self._has.set_time();
                self.r#time = value.into();
                self
            }
            /// Return a mutable reference to `time` as an `Option`
            #[inline]
            pub fn mut_time(&mut self) -> ::core::option::Option<&mut GpsTime> {
                self._has.r#time().then_some(&mut self.r#time)
            }
            /// Clear the presence of `time`
            #[inline]
            pub fn clear_time(&mut self) -> &mut Self {
                self._has.clear_time();
                self
            }
            /// Take the value of `time` and clear its presence
            #[inline]
            pub fn take_time(&mut self) -> ::core::option::Option<GpsTime> {
                let val = self
                    ._has
                    .r#time()
                    .then(|| ::core::mem::take(&mut self.r#time));
                self._has.clear_time();
                val
            }
            /// Builder method that sets the value of `time`. Useful for initializing the message.
            #[inline]
            pub fn init_time(mut self, value: GpsTime) -> Self {
                self.set_time(value);
                self
            }
            /// Return a reference to `position` as an `Option`
            #[inline]
            pub fn r#position(&self) -> ::core::option::Option<&Lla> {
                self._has.r#position().then_some(&self.r#position)
            }
            /// Set the value and presence of `position`
            #[inline]
            pub fn set_position(&mut self, value: Lla) -> &mut Self {
                self._has.set_position();
                self.r#position = value.into();
                self
            }
            /// Return a mutable reference to `position` as an `Option`
            #[inline]
            pub fn mut_position(&mut self) -> ::core::option::Option<&mut Lla> {
                self._has.r#position().then_some(&mut self.r#position)
            }
            /// Clear the presence of `position`
            #[inline]
            pub fn clear_position(&mut self) -> &mut Self {
                self._has.clear_position();
                self
            }
            /// Take the value of `position` and clear its presence
            #[inline]
            pub fn take_position(&mut self) -> ::core::option::Option<Lla> {
                let val = self
                    ._has
                    .r#position()
                    .then(|| ::core::mem::take(&mut self.r#position));
                self._has.clear_position();
                val
            }
            /// Builder method that sets the value of `position`. Useful for initializing the message.
            #[inline]
            pub fn init_position(mut self, value: Lla) -> Self {
                self.set_position(value);
                self
            }
            /// Return a reference to `position_std_m` as an `Option`
            #[inline]
            pub fn r#position_std_m(&self) -> ::core::option::Option<&Vec3> {
                self._has.r#position_std_m().then_some(&self.r#position_std_m)
            }
            /// Set the value and presence of `position_std_m`
            #[inline]
            pub fn set_position_std_m(&mut self, value: Vec3) -> &mut Self {
                self._has.set_position_std_m();
                self.r#position_std_m = value.into();
                self
            }
            /// Return a mutable reference to `position_std_m` as an `Option`
            #[inline]
            pub fn mut_position_std_m(&mut self) -> ::core::option::Option<&mut Vec3> {
                self._has.r#position_std_m().then_some(&mut self.r#position_std_m)
            }
            /// Clear the presence of `position_std_m`
            #[inline]
            pub fn clear_position_std_m(&mut self) -> &mut Self {
                self._has.clear_position_std_m();
                self
            }
            /// Take the value of `position_std_m` and clear its presence
            #[inline]
            pub fn take_position_std_m(&mut self) -> ::core::option::Option<Vec3> {
                let val = self
                    ._has
                    .r#position_std_m()
                    .then(|| ::core::mem::take(&mut self.r#position_std_m));
                self._has.clear_position_std_m();
                val
            }
            /// Builder method that sets the value of `position_std_m`. Useful for initializing the message.
            #[inline]
            pub fn init_position_std_m(mut self, value: Vec3) -> Self {
                self.set_position_std_m(value);
                self
            }
            /// Return a reference to `velocity_mps` as an `Option`
            #[inline]
            pub fn r#velocity_mps(&self) -> ::core::option::Option<&Ned> {
                self._has.r#velocity_mps().then_some(&self.r#velocity_mps)
            }
            /// Set the value and presence of `velocity_mps`
            #[inline]
            pub fn set_velocity_mps(&mut self, value: Ned) -> &mut Self {
                self._has.set_velocity_mps();
                self.r#velocity_mps = value.into();
                self
            }
            /// Return a mutable reference to `velocity_mps` as an `Option`
            #[inline]
            pub fn mut_velocity_mps(&mut self) -> ::core::option::Option<&mut Ned> {
                self._has.r#velocity_mps().then_some(&mut self.r#velocity_mps)
            }
            /// Clear the presence of `velocity_mps`
            #[inline]
            pub fn clear_velocity_mps(&mut self) -> &mut Self {
                self._has.clear_velocity_mps();
                self
            }
            /// Take the value of `velocity_mps` and clear its presence
            #[inline]
            pub fn take_velocity_mps(&mut self) -> ::core::option::Option<Ned> {
                let val = self
                    ._has
                    .r#velocity_mps()
                    .then(|| ::core::mem::take(&mut self.r#velocity_mps));
                self._has.clear_velocity_mps();
                val
            }
            /// Builder method that sets the value of `velocity_mps`. Useful for initializing the message.
            #[inline]
            pub fn init_velocity_mps(mut self, value: Ned) -> Self {
                self.set_velocity_mps(value);
                self
            }
            /// Return a reference to `velocity_std_mps` as an `Option`
            #[inline]
            pub fn r#velocity_std_mps(&self) -> ::core::option::Option<&Vec3> {
                self._has.r#velocity_std_mps().then_some(&self.r#velocity_std_mps)
            }
            /// Set the value and presence of `velocity_std_mps`
            #[inline]
            pub fn set_velocity_std_mps(&mut self, value: Vec3) -> &mut Self {
                self._has.set_velocity_std_mps();
                self.r#velocity_std_mps = value.into();
                self
            }
            /// Return a mutable reference to `velocity_std_mps` as an `Option`
            #[inline]
            pub fn mut_velocity_std_mps(&mut self) -> ::core::option::Option<&mut Vec3> {
                self._has.r#velocity_std_mps().then_some(&mut self.r#velocity_std_mps)
            }
            /// Clear the presence of `velocity_std_mps`
            #[inline]
            pub fn clear_velocity_std_mps(&mut self) -> &mut Self {
                self._has.clear_velocity_std_mps();
                self
            }
            /// Take the value of `velocity_std_mps` and clear its presence
            #[inline]
            pub fn take_velocity_std_mps(&mut self) -> ::core::option::Option<Vec3> {
                let val = self
                    ._has
                    .r#velocity_std_mps()
                    .then(|| ::core::mem::take(&mut self.r#velocity_std_mps));
                self._has.clear_velocity_std_mps();
                val
            }
            /// Builder method that sets the value of `velocity_std_mps`. Useful for initializing the message.
            #[inline]
            pub fn init_velocity_std_mps(mut self, value: Vec3) -> Self {
                self.set_velocity_std_mps(value);
                self
            }
        }
        impl ::micropb::MessageDecode for GnssFix {
            fn decode<IMPL_MICROPB_READ: ::micropb::PbRead>(
                &mut self,
                decoder: &mut ::micropb::PbDecoder<IMPL_MICROPB_READ>,
                len: usize,
            ) -> Result<(), ::micropb::DecodeError<IMPL_MICROPB_READ::Error>> {
                use ::micropb::{PbBytes, PbString, PbVec, PbMap, FieldDecode};
                let before = decoder.bytes_read();
                while decoder.bytes_read() - before < len {
                    let tag = decoder.decode_tag()?;
                    match tag.field_num() {
                        0 => return Err(::micropb::DecodeError::ZeroField),
                        1u32 => {
                            let mut_ref = &mut self.r#time;
                            {
                                mut_ref.decode_len_delimited(decoder)?;
                            };
                            self._has.set_time();
                        }
                        2u32 => {
                            let mut_ref = &mut self.r#position;
                            {
                                mut_ref.decode_len_delimited(decoder)?;
                            };
                            self._has.set_position();
                        }
                        3u32 => {
                            let mut_ref = &mut self.r#position_std_m;
                            {
                                mut_ref.decode_len_delimited(decoder)?;
                            };
                            self._has.set_position_std_m();
                        }
                        4u32 => {
                            let mut_ref = &mut self.r#velocity_mps;
                            {
                                mut_ref.decode_len_delimited(decoder)?;
                            };
                            self._has.set_velocity_mps();
                        }
                        5u32 => {
                            let mut_ref = &mut self.r#velocity_std_mps;
                            {
                                mut_ref.decode_len_delimited(decoder)?;
                            };
                            self._has.set_velocity_std_mps();
                        }
                        _ => {
                            decoder.skip_wire_value(tag.wire_type())?;
                        }
                    }
                }
                Ok(())
            }
        }
        impl ::micropb::MessageEncode for GnssFix {
            const MAX_SIZE: ::core::result::Result<usize, &'static str> = 'msg: {
                let mut max_size = 0;
                match ::micropb::const_map!(
                    ::micropb::const_map!(< GpsTime as ::micropb::MessageEncode >
                    ::MAX_SIZE, | size | ::micropb::size::sizeof_len_record(size)), |
                    size | size + 1usize
                ) {
                    ::core::result::Result::Ok(size) => {
                        max_size += size;
                    }
                    ::core::result::Result::Err(err) => {
                        break 'msg (::core::result::Result::<usize, _>::Err(err));
                    }
                }
                match ::micropb::const_map!(
                    ::micropb::const_map!(< Lla as ::micropb::MessageEncode > ::MAX_SIZE,
                    | size | ::micropb::size::sizeof_len_record(size)), | size | size +
                    1usize
                ) {
                    ::core::result::Result::Ok(size) => {
                        max_size += size;
                    }
                    ::core::result::Result::Err(err) => {
                        break 'msg (::core::result::Result::<usize, _>::Err(err));
                    }
                }
                match ::micropb::const_map!(
                    ::micropb::const_map!(< Vec3 as ::micropb::MessageEncode >
                    ::MAX_SIZE, | size | ::micropb::size::sizeof_len_record(size)), |
                    size | size + 1usize
                ) {
                    ::core::result::Result::Ok(size) => {
                        max_size += size;
                    }
                    ::core::result::Result::Err(err) => {
                        break 'msg (::core::result::Result::<usize, _>::Err(err));
                    }
                }
                match ::micropb::const_map!(
                    ::micropb::const_map!(< Ned as ::micropb::MessageEncode > ::MAX_SIZE,
                    | size | ::micropb::size::sizeof_len_record(size)), | size | size +
                    1usize
                ) {
                    ::core::result::Result::Ok(size) => {
                        max_size += size;
                    }
                    ::core::result::Result::Err(err) => {
                        break 'msg (::core::result::Result::<usize, _>::Err(err));
                    }
                }
                match ::micropb::const_map!(
                    ::micropb::const_map!(< Vec3 as ::micropb::MessageEncode >
                    ::MAX_SIZE, | size | ::micropb::size::sizeof_len_record(size)), |
                    size | size + 1usize
                ) {
                    ::core::result::Result::Ok(size) => {
                        max_size += size;
                    }
                    ::core::result::Result::Err(err) => {
                        break 'msg (::core::result::Result::<usize, _>::Err(err));
                    }
                }
                ::core::result::Result::Ok(max_size)
            };
            fn encode<IMPL_MICROPB_WRITE: ::micropb::PbWrite>(
                &self,
                encoder: &mut ::micropb::PbEncoder<IMPL_MICROPB_WRITE>,
            ) -> Result<(), IMPL_MICROPB_WRITE::Error> {
                use ::micropb::{PbMap, FieldEncode};
                {
                    if let ::core::option::Option::Some(val_ref) = self.r#time() {
                        encoder.encode_varint32(10u32)?;
                        val_ref.encode_len_delimited(encoder)?;
                    }
                }
                {
                    if let ::core::option::Option::Some(val_ref) = self.r#position() {
                        encoder.encode_varint32(18u32)?;
                        val_ref.encode_len_delimited(encoder)?;
                    }
                }
                {
                    if let ::core::option::Option::Some(val_ref) = self
                        .r#position_std_m()
                    {
                        encoder.encode_varint32(26u32)?;
                        val_ref.encode_len_delimited(encoder)?;
                    }
                }
                {
                    if let ::core::option::Option::Some(val_ref) = self.r#velocity_mps()
                    {
                        encoder.encode_varint32(34u32)?;
                        val_ref.encode_len_delimited(encoder)?;
                    }
                }
                {
                    if let ::core::option::Option::Some(val_ref) = self
                        .r#velocity_std_mps()
                    {
                        encoder.encode_varint32(42u32)?;
                        val_ref.encode_len_delimited(encoder)?;
                    }
                }
                Ok(())
            }
            fn compute_size(&self) -> usize {
                use ::micropb::{PbMap, FieldEncode};
                let mut size = 0;
                {
                    if let ::core::option::Option::Some(val_ref) = self.r#time() {
                        size
                            += 1usize
                                + ::micropb::size::sizeof_len_record(
                                    val_ref.compute_size(),
                                );
                    }
                }
                {
                    if let ::core::option::Option::Some(val_ref) = self.r#position() {
                        size
                            += 1usize
                                + ::micropb::size::sizeof_len_record(
                                    val_ref.compute_size(),
                                );
                    }
                }
                {
                    if let ::core::option::Option::Some(val_ref) = self
                        .r#position_std_m()
                    {
                        size
                            += 1usize
                                + ::micropb::size::sizeof_len_record(
                                    val_ref.compute_size(),
                                );
                    }
                }
                {
                    if let ::core::option::Option::Some(val_ref) = self.r#velocity_mps()
                    {
                        size
                            += 1usize
                                + ::micropb::size::sizeof_len_record(
                                    val_ref.compute_size(),
                                );
                    }
                }
                {
                    if let ::core::option::Option::Some(val_ref) = self
                        .r#velocity_std_mps()
                    {
                        size
                            += 1usize
                                + ::micropb::size::sizeof_len_record(
                                    val_ref.compute_size(),
                                );
                    }
                }
                size
            }
        }
        /// Inner types for `GnssFix`
        pub mod GnssFix_ {
            /// Compact bitfield for tracking presence of optional and message fields
            #[derive(Debug, Default, PartialEq, Clone, Copy)]
            pub struct _Hazzer([u8; 1]);
            impl _Hazzer {
                /// New hazzer with all fields set to off
                #[inline]
                pub const fn _new() -> Self {
                    Self([0; 1])
                }
                /// Query presence of `time`
                #[inline]
                pub const fn r#time(&self) -> bool {
                    (self.0[0] & 1) != 0
                }
                /// Set presence of `time`
                #[inline]
                pub const fn set_time(&mut self) -> &mut Self {
                    let elem = &mut self.0[0];
                    *elem |= 1;
                    self
                }
                /// Clear presence of `time`
                #[inline]
                pub const fn clear_time(&mut self) -> &mut Self {
                    let elem = &mut self.0[0];
                    *elem &= !1;
                    self
                }
                /// Builder method that sets the presence of `time`. Useful for initializing the Hazzer.
                #[inline]
                pub const fn init_time(mut self) -> Self {
                    self.set_time();
                    self
                }
                /// Query presence of `position`
                #[inline]
                pub const fn r#position(&self) -> bool {
                    (self.0[0] & 2) != 0
                }
                /// Set presence of `position`
                #[inline]
                pub const fn set_position(&mut self) -> &mut Self {
                    let elem = &mut self.0[0];
                    *elem |= 2;
                    self
                }
                /// Clear presence of `position`
                #[inline]
                pub const fn clear_position(&mut self) -> &mut Self {
                    let elem = &mut self.0[0];
                    *elem &= !2;
                    self
                }
                /// Builder method that sets the presence of `position`. Useful for initializing the Hazzer.
                #[inline]
                pub const fn init_position(mut self) -> Self {
                    self.set_position();
                    self
                }
                /// Query presence of `position_std_m`
                #[inline]
                pub const fn r#position_std_m(&self) -> bool {
                    (self.0[0] & 4) != 0
                }
                /// Set presence of `position_std_m`
                #[inline]
                pub const fn set_position_std_m(&mut self) -> &mut Self {
                    let elem = &mut self.0[0];
                    *elem |= 4;
                    self
                }
                /// Clear presence of `position_std_m`
                #[inline]
                pub const fn clear_position_std_m(&mut self) -> &mut Self {
                    let elem = &mut self.0[0];
                    *elem &= !4;
                    self
                }
                /// Builder method that sets the presence of `position_std_m`. Useful for initializing the Hazzer.
                #[inline]
                pub const fn init_position_std_m(mut self) -> Self {
                    self.set_position_std_m();
                    self
                }
                /// Query presence of `velocity_mps`
                #[inline]
                pub const fn r#velocity_mps(&self) -> bool {
                    (self.0[0] & 8) != 0
                }
                /// Set presence of `velocity_mps`
                #[inline]
                pub const fn set_velocity_mps(&mut self) -> &mut Self {
                    let elem = &mut self.0[0];
                    *elem |= 8;
                    self
                }
                /// Clear presence of `velocity_mps`
                #[inline]
                pub const fn clear_velocity_mps(&mut self) -> &mut Self {
                    let elem = &mut self.0[0];
                    *elem &= !8;
                    self
                }
                /// Builder method that sets the presence of `velocity_mps`. Useful for initializing the Hazzer.
                #[inline]
                pub const fn init_velocity_mps(mut self) -> Self {
                    self.set_velocity_mps();
                    self
                }
                /// Query presence of `velocity_std_mps`
                #[inline]
                pub const fn r#velocity_std_mps(&self) -> bool {
                    (self.0[0] & 16) != 0
                }
                /// Set presence of `velocity_std_mps`
                #[inline]
                pub const fn set_velocity_std_mps(&mut self) -> &mut Self {
                    let elem = &mut self.0[0];
                    *elem |= 16;
                    self
                }
                /// Clear presence of `velocity_std_mps`
                #[inline]
                pub const fn clear_velocity_std_mps(&mut self) -> &mut Self {
                    let elem = &mut self.0[0];
                    *elem &= !16;
                    self
                }
                /// Builder method that sets the presence of `velocity_std_mps`. Useful for initializing the Hazzer.
                #[inline]
                pub const fn init_velocity_std_mps(mut self) -> Self {
                    self.set_velocity_std_mps();
                    self
                }
            }
        }
        /// Auxiliary measurements — the aiding sources that keep a low-cost system
        /// usable through a GNSS outage.
        #[derive(Debug, Default, Clone, Copy)]
        pub struct AuxMeasurement {
            /// *Note:* The presence of this field is tracked separately in the `_has` field. It's recommended to access this field via the accessor rather than directly.
            pub r#time: GpsTime,
            pub r#measurement: ::core::option::Option<AuxMeasurement_::Measurement>,
            /// Tracks presence of optional and message fields
            pub _has: AuxMeasurement_::_Hazzer,
        }
        impl ::core::cmp::PartialEq for AuxMeasurement {
            fn eq(&self, other: &Self) -> bool {
                let mut ret = true;
                ret &= (self.r#time() == other.r#time());
                ret &= (self.r#measurement == other.r#measurement);
                ret
            }
        }
        impl AuxMeasurement {
            /// Return a reference to `time` as an `Option`
            #[inline]
            pub fn r#time(&self) -> ::core::option::Option<&GpsTime> {
                self._has.r#time().then_some(&self.r#time)
            }
            /// Set the value and presence of `time`
            #[inline]
            pub fn set_time(&mut self, value: GpsTime) -> &mut Self {
                self._has.set_time();
                self.r#time = value.into();
                self
            }
            /// Return a mutable reference to `time` as an `Option`
            #[inline]
            pub fn mut_time(&mut self) -> ::core::option::Option<&mut GpsTime> {
                self._has.r#time().then_some(&mut self.r#time)
            }
            /// Clear the presence of `time`
            #[inline]
            pub fn clear_time(&mut self) -> &mut Self {
                self._has.clear_time();
                self
            }
            /// Take the value of `time` and clear its presence
            #[inline]
            pub fn take_time(&mut self) -> ::core::option::Option<GpsTime> {
                let val = self
                    ._has
                    .r#time()
                    .then(|| ::core::mem::take(&mut self.r#time));
                self._has.clear_time();
                val
            }
            /// Builder method that sets the value of `time`. Useful for initializing the message.
            #[inline]
            pub fn init_time(mut self, value: GpsTime) -> Self {
                self.set_time(value);
                self
            }
        }
        impl ::micropb::MessageDecode for AuxMeasurement {
            fn decode<IMPL_MICROPB_READ: ::micropb::PbRead>(
                &mut self,
                decoder: &mut ::micropb::PbDecoder<IMPL_MICROPB_READ>,
                len: usize,
            ) -> Result<(), ::micropb::DecodeError<IMPL_MICROPB_READ::Error>> {
                use ::micropb::{PbBytes, PbString, PbVec, PbMap, FieldDecode};
                let before = decoder.bytes_read();
                while decoder.bytes_read() - before < len {
                    let tag = decoder.decode_tag()?;
                    match tag.field_num() {
                        0 => return Err(::micropb::DecodeError::ZeroField),
                        1u32 => {
                            let mut_ref = &mut self.r#time;
                            {
                                mut_ref.decode_len_delimited(decoder)?;
                            };
                            self._has.set_time();
                        }
                        2u32 => {
                            let mut_ref = loop {
                                if let ::core::option::Option::Some(variant) = &mut self
                                    .r#measurement
                                {
                                    if let AuxMeasurement_::Measurement::ZeroVelocity(
                                        variant,
                                    ) = &mut *variant {
                                        break &mut *variant;
                                    }
                                }
                                self.r#measurement = ::core::option::Option::Some(
                                    AuxMeasurement_::Measurement::ZeroVelocity(
                                        ::core::default::Default::default(),
                                    ),
                                );
                            };
                            mut_ref.decode_len_delimited(decoder)?;
                        }
                        3u32 => {
                            let mut_ref = loop {
                                if let ::core::option::Option::Some(variant) = &mut self
                                    .r#measurement
                                {
                                    if let AuxMeasurement_::Measurement::BarometricHeight(
                                        variant,
                                    ) = &mut *variant {
                                        break &mut *variant;
                                    }
                                }
                                self.r#measurement = ::core::option::Option::Some(
                                    AuxMeasurement_::Measurement::BarometricHeight(
                                        ::core::default::Default::default(),
                                    ),
                                );
                            };
                            mut_ref.decode_len_delimited(decoder)?;
                        }
                        4u32 => {
                            let mut_ref = loop {
                                if let ::core::option::Option::Some(variant) = &mut self
                                    .r#measurement
                                {
                                    if let AuxMeasurement_::Measurement::WheelSpeed(variant) = &mut *variant {
                                        break &mut *variant;
                                    }
                                }
                                self.r#measurement = ::core::option::Option::Some(
                                    AuxMeasurement_::Measurement::WheelSpeed(
                                        ::core::default::Default::default(),
                                    ),
                                );
                            };
                            mut_ref.decode_len_delimited(decoder)?;
                        }
                        5u32 => {
                            let mut_ref = loop {
                                if let ::core::option::Option::Some(variant) = &mut self
                                    .r#measurement
                                {
                                    if let AuxMeasurement_::Measurement::NonHolonomic(
                                        variant,
                                    ) = &mut *variant {
                                        break &mut *variant;
                                    }
                                }
                                self.r#measurement = ::core::option::Option::Some(
                                    AuxMeasurement_::Measurement::NonHolonomic(
                                        ::core::default::Default::default(),
                                    ),
                                );
                            };
                            mut_ref.decode_len_delimited(decoder)?;
                        }
                        6u32 => {
                            let mut_ref = loop {
                                if let ::core::option::Option::Some(variant) = &mut self
                                    .r#measurement
                                {
                                    if let AuxMeasurement_::Measurement::MagneticHeading(
                                        variant,
                                    ) = &mut *variant {
                                        break &mut *variant;
                                    }
                                }
                                self.r#measurement = ::core::option::Option::Some(
                                    AuxMeasurement_::Measurement::MagneticHeading(
                                        ::core::default::Default::default(),
                                    ),
                                );
                            };
                            mut_ref.decode_len_delimited(decoder)?;
                        }
                        _ => {
                            decoder.skip_wire_value(tag.wire_type())?;
                        }
                    }
                }
                Ok(())
            }
        }
        impl ::micropb::MessageEncode for AuxMeasurement {
            const MAX_SIZE: ::core::result::Result<usize, &'static str> = 'msg: {
                let mut max_size = 0;
                match ::micropb::const_map!(
                    ::micropb::const_map!(< GpsTime as ::micropb::MessageEncode >
                    ::MAX_SIZE, | size | ::micropb::size::sizeof_len_record(size)), |
                    size | size + 1usize
                ) {
                    ::core::result::Result::Ok(size) => {
                        max_size += size;
                    }
                    ::core::result::Result::Err(err) => {
                        break 'msg (::core::result::Result::<usize, _>::Err(err));
                    }
                }
                match 'oneof: {
                    let mut max_size = 0;
                    match ::micropb::const_map!(
                        ::micropb::const_map!(< ZeroVelocity as ::micropb::MessageEncode
                        > ::MAX_SIZE, | size | ::micropb::size::sizeof_len_record(size)),
                        | size | size + 1usize
                    ) {
                        ::core::result::Result::Ok(size) => {
                            if size > max_size {
                                max_size = size;
                            }
                        }
                        ::core::result::Result::Err(err) => {
                            break 'oneof (::core::result::Result::<usize, _>::Err(err));
                        }
                    }
                    match ::micropb::const_map!(
                        ::micropb::const_map!(< BarometricHeight as
                        ::micropb::MessageEncode > ::MAX_SIZE, | size |
                        ::micropb::size::sizeof_len_record(size)), | size | size + 1usize
                    ) {
                        ::core::result::Result::Ok(size) => {
                            if size > max_size {
                                max_size = size;
                            }
                        }
                        ::core::result::Result::Err(err) => {
                            break 'oneof (::core::result::Result::<usize, _>::Err(err));
                        }
                    }
                    match ::micropb::const_map!(
                        ::micropb::const_map!(< WheelSpeed as ::micropb::MessageEncode >
                        ::MAX_SIZE, | size | ::micropb::size::sizeof_len_record(size)), |
                        size | size + 1usize
                    ) {
                        ::core::result::Result::Ok(size) => {
                            if size > max_size {
                                max_size = size;
                            }
                        }
                        ::core::result::Result::Err(err) => {
                            break 'oneof (::core::result::Result::<usize, _>::Err(err));
                        }
                    }
                    match ::micropb::const_map!(
                        ::micropb::const_map!(< NonHolonomic as ::micropb::MessageEncode
                        > ::MAX_SIZE, | size | ::micropb::size::sizeof_len_record(size)),
                        | size | size + 1usize
                    ) {
                        ::core::result::Result::Ok(size) => {
                            if size > max_size {
                                max_size = size;
                            }
                        }
                        ::core::result::Result::Err(err) => {
                            break 'oneof (::core::result::Result::<usize, _>::Err(err));
                        }
                    }
                    match ::micropb::const_map!(
                        ::micropb::const_map!(< MagneticHeading as
                        ::micropb::MessageEncode > ::MAX_SIZE, | size |
                        ::micropb::size::sizeof_len_record(size)), | size | size + 1usize
                    ) {
                        ::core::result::Result::Ok(size) => {
                            if size > max_size {
                                max_size = size;
                            }
                        }
                        ::core::result::Result::Err(err) => {
                            break 'oneof (::core::result::Result::<usize, _>::Err(err));
                        }
                    }
                    ::core::result::Result::Ok(max_size)
                } {
                    ::core::result::Result::Ok(size) => {
                        max_size += size;
                    }
                    ::core::result::Result::Err(err) => {
                        break 'msg (::core::result::Result::<usize, _>::Err(err));
                    }
                }
                ::core::result::Result::Ok(max_size)
            };
            fn encode<IMPL_MICROPB_WRITE: ::micropb::PbWrite>(
                &self,
                encoder: &mut ::micropb::PbEncoder<IMPL_MICROPB_WRITE>,
            ) -> Result<(), IMPL_MICROPB_WRITE::Error> {
                use ::micropb::{PbMap, FieldEncode};
                {
                    if let ::core::option::Option::Some(val_ref) = self.r#time() {
                        encoder.encode_varint32(10u32)?;
                        val_ref.encode_len_delimited(encoder)?;
                    }
                }
                if let Some(oneof) = &self.r#measurement {
                    match &*oneof {
                        AuxMeasurement_::Measurement::ZeroVelocity(val_ref) => {
                            let val_ref = &*val_ref;
                            encoder.encode_varint32(18u32)?;
                            val_ref.encode_len_delimited(encoder)?;
                        }
                        AuxMeasurement_::Measurement::BarometricHeight(val_ref) => {
                            let val_ref = &*val_ref;
                            encoder.encode_varint32(26u32)?;
                            val_ref.encode_len_delimited(encoder)?;
                        }
                        AuxMeasurement_::Measurement::WheelSpeed(val_ref) => {
                            let val_ref = &*val_ref;
                            encoder.encode_varint32(34u32)?;
                            val_ref.encode_len_delimited(encoder)?;
                        }
                        AuxMeasurement_::Measurement::NonHolonomic(val_ref) => {
                            let val_ref = &*val_ref;
                            encoder.encode_varint32(42u32)?;
                            val_ref.encode_len_delimited(encoder)?;
                        }
                        AuxMeasurement_::Measurement::MagneticHeading(val_ref) => {
                            let val_ref = &*val_ref;
                            encoder.encode_varint32(50u32)?;
                            val_ref.encode_len_delimited(encoder)?;
                        }
                    }
                }
                Ok(())
            }
            fn compute_size(&self) -> usize {
                use ::micropb::{PbMap, FieldEncode};
                let mut size = 0;
                {
                    if let ::core::option::Option::Some(val_ref) = self.r#time() {
                        size
                            += 1usize
                                + ::micropb::size::sizeof_len_record(
                                    val_ref.compute_size(),
                                );
                    }
                }
                if let Some(oneof) = &self.r#measurement {
                    match &*oneof {
                        AuxMeasurement_::Measurement::ZeroVelocity(val_ref) => {
                            let val_ref = &*val_ref;
                            size
                                += 1usize
                                    + ::micropb::size::sizeof_len_record(
                                        val_ref.compute_size(),
                                    );
                        }
                        AuxMeasurement_::Measurement::BarometricHeight(val_ref) => {
                            let val_ref = &*val_ref;
                            size
                                += 1usize
                                    + ::micropb::size::sizeof_len_record(
                                        val_ref.compute_size(),
                                    );
                        }
                        AuxMeasurement_::Measurement::WheelSpeed(val_ref) => {
                            let val_ref = &*val_ref;
                            size
                                += 1usize
                                    + ::micropb::size::sizeof_len_record(
                                        val_ref.compute_size(),
                                    );
                        }
                        AuxMeasurement_::Measurement::NonHolonomic(val_ref) => {
                            let val_ref = &*val_ref;
                            size
                                += 1usize
                                    + ::micropb::size::sizeof_len_record(
                                        val_ref.compute_size(),
                                    );
                        }
                        AuxMeasurement_::Measurement::MagneticHeading(val_ref) => {
                            let val_ref = &*val_ref;
                            size
                                += 1usize
                                    + ::micropb::size::sizeof_len_record(
                                        val_ref.compute_size(),
                                    );
                        }
                    }
                }
                size
            }
        }
        /// Inner types for `AuxMeasurement`
        pub mod AuxMeasurement_ {
            #[derive(Debug, PartialEq, Clone, Copy)]
            pub enum Measurement {
                /// Detected stationarity: velocity is zero in all three axes.
                ZeroVelocity(super::ZeroVelocity),
                /// Barometric height, which bounds the unstable INS vertical channel.
                BarometricHeight(super::BarometricHeight),
                /// Odometer / wheel speed along the body forward axis.
                WheelSpeed(super::WheelSpeed),
                /// Non-holonomic constraints for a wheeled vehicle.
                NonHolonomic(super::NonHolonomic),
                /// Magnetometer heading.
                MagneticHeading(super::MagneticHeading),
            }
            /// Compact bitfield for tracking presence of optional and message fields
            #[derive(Debug, Default, PartialEq, Clone, Copy)]
            pub struct _Hazzer([u8; 1]);
            impl _Hazzer {
                /// New hazzer with all fields set to off
                #[inline]
                pub const fn _new() -> Self {
                    Self([0; 1])
                }
                /// Query presence of `time`
                #[inline]
                pub const fn r#time(&self) -> bool {
                    (self.0[0] & 1) != 0
                }
                /// Set presence of `time`
                #[inline]
                pub const fn set_time(&mut self) -> &mut Self {
                    let elem = &mut self.0[0];
                    *elem |= 1;
                    self
                }
                /// Clear presence of `time`
                #[inline]
                pub const fn clear_time(&mut self) -> &mut Self {
                    let elem = &mut self.0[0];
                    *elem &= !1;
                    self
                }
                /// Builder method that sets the presence of `time`. Useful for initializing the Hazzer.
                #[inline]
                pub const fn init_time(mut self) -> Self {
                    self.set_time();
                    self
                }
            }
        }
        #[derive(Debug, Default, Clone, Copy)]
        pub struct ZeroVelocity {
            /// One-sigma on the assumed-zero velocity, m/s.
            ///
            /// *Note:* The presence of this field is tracked separately in the `_has` field. It's recommended to access this field via the accessor rather than directly.
            pub r#velocity_std_mps: Vec3,
            /// Tracks presence of optional and message fields
            pub _has: ZeroVelocity_::_Hazzer,
        }
        impl ::core::cmp::PartialEq for ZeroVelocity {
            fn eq(&self, other: &Self) -> bool {
                let mut ret = true;
                ret &= (self.r#velocity_std_mps() == other.r#velocity_std_mps());
                ret
            }
        }
        impl ZeroVelocity {
            /// Return a reference to `velocity_std_mps` as an `Option`
            #[inline]
            pub fn r#velocity_std_mps(&self) -> ::core::option::Option<&Vec3> {
                self._has.r#velocity_std_mps().then_some(&self.r#velocity_std_mps)
            }
            /// Set the value and presence of `velocity_std_mps`
            #[inline]
            pub fn set_velocity_std_mps(&mut self, value: Vec3) -> &mut Self {
                self._has.set_velocity_std_mps();
                self.r#velocity_std_mps = value.into();
                self
            }
            /// Return a mutable reference to `velocity_std_mps` as an `Option`
            #[inline]
            pub fn mut_velocity_std_mps(&mut self) -> ::core::option::Option<&mut Vec3> {
                self._has.r#velocity_std_mps().then_some(&mut self.r#velocity_std_mps)
            }
            /// Clear the presence of `velocity_std_mps`
            #[inline]
            pub fn clear_velocity_std_mps(&mut self) -> &mut Self {
                self._has.clear_velocity_std_mps();
                self
            }
            /// Take the value of `velocity_std_mps` and clear its presence
            #[inline]
            pub fn take_velocity_std_mps(&mut self) -> ::core::option::Option<Vec3> {
                let val = self
                    ._has
                    .r#velocity_std_mps()
                    .then(|| ::core::mem::take(&mut self.r#velocity_std_mps));
                self._has.clear_velocity_std_mps();
                val
            }
            /// Builder method that sets the value of `velocity_std_mps`. Useful for initializing the message.
            #[inline]
            pub fn init_velocity_std_mps(mut self, value: Vec3) -> Self {
                self.set_velocity_std_mps(value);
                self
            }
        }
        impl ::micropb::MessageDecode for ZeroVelocity {
            fn decode<IMPL_MICROPB_READ: ::micropb::PbRead>(
                &mut self,
                decoder: &mut ::micropb::PbDecoder<IMPL_MICROPB_READ>,
                len: usize,
            ) -> Result<(), ::micropb::DecodeError<IMPL_MICROPB_READ::Error>> {
                use ::micropb::{PbBytes, PbString, PbVec, PbMap, FieldDecode};
                let before = decoder.bytes_read();
                while decoder.bytes_read() - before < len {
                    let tag = decoder.decode_tag()?;
                    match tag.field_num() {
                        0 => return Err(::micropb::DecodeError::ZeroField),
                        1u32 => {
                            let mut_ref = &mut self.r#velocity_std_mps;
                            {
                                mut_ref.decode_len_delimited(decoder)?;
                            };
                            self._has.set_velocity_std_mps();
                        }
                        _ => {
                            decoder.skip_wire_value(tag.wire_type())?;
                        }
                    }
                }
                Ok(())
            }
        }
        impl ::micropb::MessageEncode for ZeroVelocity {
            const MAX_SIZE: ::core::result::Result<usize, &'static str> = 'msg: {
                let mut max_size = 0;
                match ::micropb::const_map!(
                    ::micropb::const_map!(< Vec3 as ::micropb::MessageEncode >
                    ::MAX_SIZE, | size | ::micropb::size::sizeof_len_record(size)), |
                    size | size + 1usize
                ) {
                    ::core::result::Result::Ok(size) => {
                        max_size += size;
                    }
                    ::core::result::Result::Err(err) => {
                        break 'msg (::core::result::Result::<usize, _>::Err(err));
                    }
                }
                ::core::result::Result::Ok(max_size)
            };
            fn encode<IMPL_MICROPB_WRITE: ::micropb::PbWrite>(
                &self,
                encoder: &mut ::micropb::PbEncoder<IMPL_MICROPB_WRITE>,
            ) -> Result<(), IMPL_MICROPB_WRITE::Error> {
                use ::micropb::{PbMap, FieldEncode};
                {
                    if let ::core::option::Option::Some(val_ref) = self
                        .r#velocity_std_mps()
                    {
                        encoder.encode_varint32(10u32)?;
                        val_ref.encode_len_delimited(encoder)?;
                    }
                }
                Ok(())
            }
            fn compute_size(&self) -> usize {
                use ::micropb::{PbMap, FieldEncode};
                let mut size = 0;
                {
                    if let ::core::option::Option::Some(val_ref) = self
                        .r#velocity_std_mps()
                    {
                        size
                            += 1usize
                                + ::micropb::size::sizeof_len_record(
                                    val_ref.compute_size(),
                                );
                    }
                }
                size
            }
        }
        /// Inner types for `ZeroVelocity`
        pub mod ZeroVelocity_ {
            /// Compact bitfield for tracking presence of optional and message fields
            #[derive(Debug, Default, PartialEq, Clone, Copy)]
            pub struct _Hazzer([u8; 1]);
            impl _Hazzer {
                /// New hazzer with all fields set to off
                #[inline]
                pub const fn _new() -> Self {
                    Self([0; 1])
                }
                /// Query presence of `velocity_std_mps`
                #[inline]
                pub const fn r#velocity_std_mps(&self) -> bool {
                    (self.0[0] & 1) != 0
                }
                /// Set presence of `velocity_std_mps`
                #[inline]
                pub const fn set_velocity_std_mps(&mut self) -> &mut Self {
                    let elem = &mut self.0[0];
                    *elem |= 1;
                    self
                }
                /// Clear presence of `velocity_std_mps`
                #[inline]
                pub const fn clear_velocity_std_mps(&mut self) -> &mut Self {
                    let elem = &mut self.0[0];
                    *elem &= !1;
                    self
                }
                /// Builder method that sets the presence of `velocity_std_mps`. Useful for initializing the Hazzer.
                #[inline]
                pub const fn init_velocity_std_mps(mut self) -> Self {
                    self.set_velocity_std_mps();
                    self
                }
            }
        }
        #[derive(Debug, Default, PartialEq, Clone, Copy)]
        pub struct BarometricHeight {
            /// Height above the WGS-84 ellipsoid, metres.
            pub r#height_m: f64,
            pub r#height_std_m: f64,
        }
        impl BarometricHeight {
            /// Return a reference to `height_m`
            #[inline]
            pub fn r#height_m(&self) -> &f64 {
                &self.r#height_m
            }
            /// Return a mutable reference to `height_m`
            #[inline]
            pub fn mut_height_m(&mut self) -> &mut f64 {
                &mut self.r#height_m
            }
            /// Set the value of `height_m`
            #[inline]
            pub fn set_height_m(&mut self, value: f64) -> &mut Self {
                self.r#height_m = value.into();
                self
            }
            /// Builder method that sets the value of `height_m`. Useful for initializing the message.
            #[inline]
            pub fn init_height_m(mut self, value: f64) -> Self {
                self.r#height_m = value.into();
                self
            }
            /// Return a reference to `height_std_m`
            #[inline]
            pub fn r#height_std_m(&self) -> &f64 {
                &self.r#height_std_m
            }
            /// Return a mutable reference to `height_std_m`
            #[inline]
            pub fn mut_height_std_m(&mut self) -> &mut f64 {
                &mut self.r#height_std_m
            }
            /// Set the value of `height_std_m`
            #[inline]
            pub fn set_height_std_m(&mut self, value: f64) -> &mut Self {
                self.r#height_std_m = value.into();
                self
            }
            /// Builder method that sets the value of `height_std_m`. Useful for initializing the message.
            #[inline]
            pub fn init_height_std_m(mut self, value: f64) -> Self {
                self.r#height_std_m = value.into();
                self
            }
        }
        impl ::micropb::MessageDecode for BarometricHeight {
            fn decode<IMPL_MICROPB_READ: ::micropb::PbRead>(
                &mut self,
                decoder: &mut ::micropb::PbDecoder<IMPL_MICROPB_READ>,
                len: usize,
            ) -> Result<(), ::micropb::DecodeError<IMPL_MICROPB_READ::Error>> {
                use ::micropb::{PbBytes, PbString, PbVec, PbMap, FieldDecode};
                let before = decoder.bytes_read();
                while decoder.bytes_read() - before < len {
                    let tag = decoder.decode_tag()?;
                    match tag.field_num() {
                        0 => return Err(::micropb::DecodeError::ZeroField),
                        1u32 => {
                            let mut_ref = &mut self.r#height_m;
                            {
                                let val = decoder.decode_double()?;
                                let val_ref = &val;
                                if *val_ref != 0.0 {
                                    *mut_ref = val as _;
                                }
                            };
                        }
                        2u32 => {
                            let mut_ref = &mut self.r#height_std_m;
                            {
                                let val = decoder.decode_double()?;
                                let val_ref = &val;
                                if *val_ref != 0.0 {
                                    *mut_ref = val as _;
                                }
                            };
                        }
                        _ => {
                            decoder.skip_wire_value(tag.wire_type())?;
                        }
                    }
                }
                Ok(())
            }
        }
        impl ::micropb::MessageEncode for BarometricHeight {
            const MAX_SIZE: ::core::result::Result<usize, &'static str> = 'msg: {
                let mut max_size = 0;
                match ::micropb::const_map!(
                    ::core::result::Result::Ok(8usize), | size | size + 1usize
                ) {
                    ::core::result::Result::Ok(size) => {
                        max_size += size;
                    }
                    ::core::result::Result::Err(err) => {
                        break 'msg (::core::result::Result::<usize, _>::Err(err));
                    }
                }
                match ::micropb::const_map!(
                    ::core::result::Result::Ok(8usize), | size | size + 1usize
                ) {
                    ::core::result::Result::Ok(size) => {
                        max_size += size;
                    }
                    ::core::result::Result::Err(err) => {
                        break 'msg (::core::result::Result::<usize, _>::Err(err));
                    }
                }
                ::core::result::Result::Ok(max_size)
            };
            fn encode<IMPL_MICROPB_WRITE: ::micropb::PbWrite>(
                &self,
                encoder: &mut ::micropb::PbEncoder<IMPL_MICROPB_WRITE>,
            ) -> Result<(), IMPL_MICROPB_WRITE::Error> {
                use ::micropb::{PbMap, FieldEncode};
                {
                    let val_ref = &self.r#height_m;
                    if *val_ref != 0.0 {
                        encoder.encode_varint32(9u32)?;
                        encoder.encode_double(*val_ref)?;
                    }
                }
                {
                    let val_ref = &self.r#height_std_m;
                    if *val_ref != 0.0 {
                        encoder.encode_varint32(17u32)?;
                        encoder.encode_double(*val_ref)?;
                    }
                }
                Ok(())
            }
            fn compute_size(&self) -> usize {
                use ::micropb::{PbMap, FieldEncode};
                let mut size = 0;
                {
                    let val_ref = &self.r#height_m;
                    if *val_ref != 0.0 {
                        size += 1usize + 8;
                    }
                }
                {
                    let val_ref = &self.r#height_std_m;
                    if *val_ref != 0.0 {
                        size += 1usize + 8;
                    }
                }
                size
            }
        }
        #[derive(Debug, Default, PartialEq, Clone, Copy)]
        pub struct WheelSpeed {
            /// Speed along the body forward axis, m/s.
            pub r#speed_mps: f64,
            pub r#speed_std_mps: f64,
        }
        impl WheelSpeed {
            /// Return a reference to `speed_mps`
            #[inline]
            pub fn r#speed_mps(&self) -> &f64 {
                &self.r#speed_mps
            }
            /// Return a mutable reference to `speed_mps`
            #[inline]
            pub fn mut_speed_mps(&mut self) -> &mut f64 {
                &mut self.r#speed_mps
            }
            /// Set the value of `speed_mps`
            #[inline]
            pub fn set_speed_mps(&mut self, value: f64) -> &mut Self {
                self.r#speed_mps = value.into();
                self
            }
            /// Builder method that sets the value of `speed_mps`. Useful for initializing the message.
            #[inline]
            pub fn init_speed_mps(mut self, value: f64) -> Self {
                self.r#speed_mps = value.into();
                self
            }
            /// Return a reference to `speed_std_mps`
            #[inline]
            pub fn r#speed_std_mps(&self) -> &f64 {
                &self.r#speed_std_mps
            }
            /// Return a mutable reference to `speed_std_mps`
            #[inline]
            pub fn mut_speed_std_mps(&mut self) -> &mut f64 {
                &mut self.r#speed_std_mps
            }
            /// Set the value of `speed_std_mps`
            #[inline]
            pub fn set_speed_std_mps(&mut self, value: f64) -> &mut Self {
                self.r#speed_std_mps = value.into();
                self
            }
            /// Builder method that sets the value of `speed_std_mps`. Useful for initializing the message.
            #[inline]
            pub fn init_speed_std_mps(mut self, value: f64) -> Self {
                self.r#speed_std_mps = value.into();
                self
            }
        }
        impl ::micropb::MessageDecode for WheelSpeed {
            fn decode<IMPL_MICROPB_READ: ::micropb::PbRead>(
                &mut self,
                decoder: &mut ::micropb::PbDecoder<IMPL_MICROPB_READ>,
                len: usize,
            ) -> Result<(), ::micropb::DecodeError<IMPL_MICROPB_READ::Error>> {
                use ::micropb::{PbBytes, PbString, PbVec, PbMap, FieldDecode};
                let before = decoder.bytes_read();
                while decoder.bytes_read() - before < len {
                    let tag = decoder.decode_tag()?;
                    match tag.field_num() {
                        0 => return Err(::micropb::DecodeError::ZeroField),
                        1u32 => {
                            let mut_ref = &mut self.r#speed_mps;
                            {
                                let val = decoder.decode_double()?;
                                let val_ref = &val;
                                if *val_ref != 0.0 {
                                    *mut_ref = val as _;
                                }
                            };
                        }
                        2u32 => {
                            let mut_ref = &mut self.r#speed_std_mps;
                            {
                                let val = decoder.decode_double()?;
                                let val_ref = &val;
                                if *val_ref != 0.0 {
                                    *mut_ref = val as _;
                                }
                            };
                        }
                        _ => {
                            decoder.skip_wire_value(tag.wire_type())?;
                        }
                    }
                }
                Ok(())
            }
        }
        impl ::micropb::MessageEncode for WheelSpeed {
            const MAX_SIZE: ::core::result::Result<usize, &'static str> = 'msg: {
                let mut max_size = 0;
                match ::micropb::const_map!(
                    ::core::result::Result::Ok(8usize), | size | size + 1usize
                ) {
                    ::core::result::Result::Ok(size) => {
                        max_size += size;
                    }
                    ::core::result::Result::Err(err) => {
                        break 'msg (::core::result::Result::<usize, _>::Err(err));
                    }
                }
                match ::micropb::const_map!(
                    ::core::result::Result::Ok(8usize), | size | size + 1usize
                ) {
                    ::core::result::Result::Ok(size) => {
                        max_size += size;
                    }
                    ::core::result::Result::Err(err) => {
                        break 'msg (::core::result::Result::<usize, _>::Err(err));
                    }
                }
                ::core::result::Result::Ok(max_size)
            };
            fn encode<IMPL_MICROPB_WRITE: ::micropb::PbWrite>(
                &self,
                encoder: &mut ::micropb::PbEncoder<IMPL_MICROPB_WRITE>,
            ) -> Result<(), IMPL_MICROPB_WRITE::Error> {
                use ::micropb::{PbMap, FieldEncode};
                {
                    let val_ref = &self.r#speed_mps;
                    if *val_ref != 0.0 {
                        encoder.encode_varint32(9u32)?;
                        encoder.encode_double(*val_ref)?;
                    }
                }
                {
                    let val_ref = &self.r#speed_std_mps;
                    if *val_ref != 0.0 {
                        encoder.encode_varint32(17u32)?;
                        encoder.encode_double(*val_ref)?;
                    }
                }
                Ok(())
            }
            fn compute_size(&self) -> usize {
                use ::micropb::{PbMap, FieldEncode};
                let mut size = 0;
                {
                    let val_ref = &self.r#speed_mps;
                    if *val_ref != 0.0 {
                        size += 1usize + 8;
                    }
                }
                {
                    let val_ref = &self.r#speed_std_mps;
                    if *val_ref != 0.0 {
                        size += 1usize + 8;
                    }
                }
                size
            }
        }
        /// A wheeled vehicle does not slip sideways and does not leave the ground, so
        /// the body-frame lateral and vertical velocities are zero.
        #[derive(Debug, Default, PartialEq, Clone, Copy)]
        pub struct NonHolonomic {
            /// One-sigma slip allowance, lateral (body y), m/s.
            pub r#lateral_std_mps: f64,
            /// One-sigma allowance, vertical (body z), m/s.
            pub r#vertical_std_mps: f64,
            /// Minimum speed below which the constraint carries no information and must
            /// not be applied, m/s.
            pub r#min_speed_mps: f64,
        }
        impl NonHolonomic {
            /// Return a reference to `lateral_std_mps`
            #[inline]
            pub fn r#lateral_std_mps(&self) -> &f64 {
                &self.r#lateral_std_mps
            }
            /// Return a mutable reference to `lateral_std_mps`
            #[inline]
            pub fn mut_lateral_std_mps(&mut self) -> &mut f64 {
                &mut self.r#lateral_std_mps
            }
            /// Set the value of `lateral_std_mps`
            #[inline]
            pub fn set_lateral_std_mps(&mut self, value: f64) -> &mut Self {
                self.r#lateral_std_mps = value.into();
                self
            }
            /// Builder method that sets the value of `lateral_std_mps`. Useful for initializing the message.
            #[inline]
            pub fn init_lateral_std_mps(mut self, value: f64) -> Self {
                self.r#lateral_std_mps = value.into();
                self
            }
            /// Return a reference to `vertical_std_mps`
            #[inline]
            pub fn r#vertical_std_mps(&self) -> &f64 {
                &self.r#vertical_std_mps
            }
            /// Return a mutable reference to `vertical_std_mps`
            #[inline]
            pub fn mut_vertical_std_mps(&mut self) -> &mut f64 {
                &mut self.r#vertical_std_mps
            }
            /// Set the value of `vertical_std_mps`
            #[inline]
            pub fn set_vertical_std_mps(&mut self, value: f64) -> &mut Self {
                self.r#vertical_std_mps = value.into();
                self
            }
            /// Builder method that sets the value of `vertical_std_mps`. Useful for initializing the message.
            #[inline]
            pub fn init_vertical_std_mps(mut self, value: f64) -> Self {
                self.r#vertical_std_mps = value.into();
                self
            }
            /// Return a reference to `min_speed_mps`
            #[inline]
            pub fn r#min_speed_mps(&self) -> &f64 {
                &self.r#min_speed_mps
            }
            /// Return a mutable reference to `min_speed_mps`
            #[inline]
            pub fn mut_min_speed_mps(&mut self) -> &mut f64 {
                &mut self.r#min_speed_mps
            }
            /// Set the value of `min_speed_mps`
            #[inline]
            pub fn set_min_speed_mps(&mut self, value: f64) -> &mut Self {
                self.r#min_speed_mps = value.into();
                self
            }
            /// Builder method that sets the value of `min_speed_mps`. Useful for initializing the message.
            #[inline]
            pub fn init_min_speed_mps(mut self, value: f64) -> Self {
                self.r#min_speed_mps = value.into();
                self
            }
        }
        impl ::micropb::MessageDecode for NonHolonomic {
            fn decode<IMPL_MICROPB_READ: ::micropb::PbRead>(
                &mut self,
                decoder: &mut ::micropb::PbDecoder<IMPL_MICROPB_READ>,
                len: usize,
            ) -> Result<(), ::micropb::DecodeError<IMPL_MICROPB_READ::Error>> {
                use ::micropb::{PbBytes, PbString, PbVec, PbMap, FieldDecode};
                let before = decoder.bytes_read();
                while decoder.bytes_read() - before < len {
                    let tag = decoder.decode_tag()?;
                    match tag.field_num() {
                        0 => return Err(::micropb::DecodeError::ZeroField),
                        1u32 => {
                            let mut_ref = &mut self.r#lateral_std_mps;
                            {
                                let val = decoder.decode_double()?;
                                let val_ref = &val;
                                if *val_ref != 0.0 {
                                    *mut_ref = val as _;
                                }
                            };
                        }
                        2u32 => {
                            let mut_ref = &mut self.r#vertical_std_mps;
                            {
                                let val = decoder.decode_double()?;
                                let val_ref = &val;
                                if *val_ref != 0.0 {
                                    *mut_ref = val as _;
                                }
                            };
                        }
                        3u32 => {
                            let mut_ref = &mut self.r#min_speed_mps;
                            {
                                let val = decoder.decode_double()?;
                                let val_ref = &val;
                                if *val_ref != 0.0 {
                                    *mut_ref = val as _;
                                }
                            };
                        }
                        _ => {
                            decoder.skip_wire_value(tag.wire_type())?;
                        }
                    }
                }
                Ok(())
            }
        }
        impl ::micropb::MessageEncode for NonHolonomic {
            const MAX_SIZE: ::core::result::Result<usize, &'static str> = 'msg: {
                let mut max_size = 0;
                match ::micropb::const_map!(
                    ::core::result::Result::Ok(8usize), | size | size + 1usize
                ) {
                    ::core::result::Result::Ok(size) => {
                        max_size += size;
                    }
                    ::core::result::Result::Err(err) => {
                        break 'msg (::core::result::Result::<usize, _>::Err(err));
                    }
                }
                match ::micropb::const_map!(
                    ::core::result::Result::Ok(8usize), | size | size + 1usize
                ) {
                    ::core::result::Result::Ok(size) => {
                        max_size += size;
                    }
                    ::core::result::Result::Err(err) => {
                        break 'msg (::core::result::Result::<usize, _>::Err(err));
                    }
                }
                match ::micropb::const_map!(
                    ::core::result::Result::Ok(8usize), | size | size + 1usize
                ) {
                    ::core::result::Result::Ok(size) => {
                        max_size += size;
                    }
                    ::core::result::Result::Err(err) => {
                        break 'msg (::core::result::Result::<usize, _>::Err(err));
                    }
                }
                ::core::result::Result::Ok(max_size)
            };
            fn encode<IMPL_MICROPB_WRITE: ::micropb::PbWrite>(
                &self,
                encoder: &mut ::micropb::PbEncoder<IMPL_MICROPB_WRITE>,
            ) -> Result<(), IMPL_MICROPB_WRITE::Error> {
                use ::micropb::{PbMap, FieldEncode};
                {
                    let val_ref = &self.r#lateral_std_mps;
                    if *val_ref != 0.0 {
                        encoder.encode_varint32(9u32)?;
                        encoder.encode_double(*val_ref)?;
                    }
                }
                {
                    let val_ref = &self.r#vertical_std_mps;
                    if *val_ref != 0.0 {
                        encoder.encode_varint32(17u32)?;
                        encoder.encode_double(*val_ref)?;
                    }
                }
                {
                    let val_ref = &self.r#min_speed_mps;
                    if *val_ref != 0.0 {
                        encoder.encode_varint32(25u32)?;
                        encoder.encode_double(*val_ref)?;
                    }
                }
                Ok(())
            }
            fn compute_size(&self) -> usize {
                use ::micropb::{PbMap, FieldEncode};
                let mut size = 0;
                {
                    let val_ref = &self.r#lateral_std_mps;
                    if *val_ref != 0.0 {
                        size += 1usize + 8;
                    }
                }
                {
                    let val_ref = &self.r#vertical_std_mps;
                    if *val_ref != 0.0 {
                        size += 1usize + 8;
                    }
                }
                {
                    let val_ref = &self.r#min_speed_mps;
                    if *val_ref != 0.0 {
                        size += 1usize + 8;
                    }
                }
                size
            }
        }
        #[derive(Debug, Default, PartialEq, Clone, Copy)]
        pub struct MagneticHeading {
            /// TRUE heading, radians clockwise from north. Magnetic declination must
            /// already have been applied — feeding magnetic heading here produces a bias
            /// equal to the local declination, which reaches tens of degrees.
            pub r#heading_rad: f64,
            pub r#heading_std_rad: f64,
        }
        impl MagneticHeading {
            /// Return a reference to `heading_rad`
            #[inline]
            pub fn r#heading_rad(&self) -> &f64 {
                &self.r#heading_rad
            }
            /// Return a mutable reference to `heading_rad`
            #[inline]
            pub fn mut_heading_rad(&mut self) -> &mut f64 {
                &mut self.r#heading_rad
            }
            /// Set the value of `heading_rad`
            #[inline]
            pub fn set_heading_rad(&mut self, value: f64) -> &mut Self {
                self.r#heading_rad = value.into();
                self
            }
            /// Builder method that sets the value of `heading_rad`. Useful for initializing the message.
            #[inline]
            pub fn init_heading_rad(mut self, value: f64) -> Self {
                self.r#heading_rad = value.into();
                self
            }
            /// Return a reference to `heading_std_rad`
            #[inline]
            pub fn r#heading_std_rad(&self) -> &f64 {
                &self.r#heading_std_rad
            }
            /// Return a mutable reference to `heading_std_rad`
            #[inline]
            pub fn mut_heading_std_rad(&mut self) -> &mut f64 {
                &mut self.r#heading_std_rad
            }
            /// Set the value of `heading_std_rad`
            #[inline]
            pub fn set_heading_std_rad(&mut self, value: f64) -> &mut Self {
                self.r#heading_std_rad = value.into();
                self
            }
            /// Builder method that sets the value of `heading_std_rad`. Useful for initializing the message.
            #[inline]
            pub fn init_heading_std_rad(mut self, value: f64) -> Self {
                self.r#heading_std_rad = value.into();
                self
            }
        }
        impl ::micropb::MessageDecode for MagneticHeading {
            fn decode<IMPL_MICROPB_READ: ::micropb::PbRead>(
                &mut self,
                decoder: &mut ::micropb::PbDecoder<IMPL_MICROPB_READ>,
                len: usize,
            ) -> Result<(), ::micropb::DecodeError<IMPL_MICROPB_READ::Error>> {
                use ::micropb::{PbBytes, PbString, PbVec, PbMap, FieldDecode};
                let before = decoder.bytes_read();
                while decoder.bytes_read() - before < len {
                    let tag = decoder.decode_tag()?;
                    match tag.field_num() {
                        0 => return Err(::micropb::DecodeError::ZeroField),
                        1u32 => {
                            let mut_ref = &mut self.r#heading_rad;
                            {
                                let val = decoder.decode_double()?;
                                let val_ref = &val;
                                if *val_ref != 0.0 {
                                    *mut_ref = val as _;
                                }
                            };
                        }
                        2u32 => {
                            let mut_ref = &mut self.r#heading_std_rad;
                            {
                                let val = decoder.decode_double()?;
                                let val_ref = &val;
                                if *val_ref != 0.0 {
                                    *mut_ref = val as _;
                                }
                            };
                        }
                        _ => {
                            decoder.skip_wire_value(tag.wire_type())?;
                        }
                    }
                }
                Ok(())
            }
        }
        impl ::micropb::MessageEncode for MagneticHeading {
            const MAX_SIZE: ::core::result::Result<usize, &'static str> = 'msg: {
                let mut max_size = 0;
                match ::micropb::const_map!(
                    ::core::result::Result::Ok(8usize), | size | size + 1usize
                ) {
                    ::core::result::Result::Ok(size) => {
                        max_size += size;
                    }
                    ::core::result::Result::Err(err) => {
                        break 'msg (::core::result::Result::<usize, _>::Err(err));
                    }
                }
                match ::micropb::const_map!(
                    ::core::result::Result::Ok(8usize), | size | size + 1usize
                ) {
                    ::core::result::Result::Ok(size) => {
                        max_size += size;
                    }
                    ::core::result::Result::Err(err) => {
                        break 'msg (::core::result::Result::<usize, _>::Err(err));
                    }
                }
                ::core::result::Result::Ok(max_size)
            };
            fn encode<IMPL_MICROPB_WRITE: ::micropb::PbWrite>(
                &self,
                encoder: &mut ::micropb::PbEncoder<IMPL_MICROPB_WRITE>,
            ) -> Result<(), IMPL_MICROPB_WRITE::Error> {
                use ::micropb::{PbMap, FieldEncode};
                {
                    let val_ref = &self.r#heading_rad;
                    if *val_ref != 0.0 {
                        encoder.encode_varint32(9u32)?;
                        encoder.encode_double(*val_ref)?;
                    }
                }
                {
                    let val_ref = &self.r#heading_std_rad;
                    if *val_ref != 0.0 {
                        encoder.encode_varint32(17u32)?;
                        encoder.encode_double(*val_ref)?;
                    }
                }
                Ok(())
            }
            fn compute_size(&self) -> usize {
                use ::micropb::{PbMap, FieldEncode};
                let mut size = 0;
                {
                    let val_ref = &self.r#heading_rad;
                    if *val_ref != 0.0 {
                        size += 1usize + 8;
                    }
                }
                {
                    let val_ref = &self.r#heading_std_rad;
                    if *val_ref != 0.0 {
                        size += 1usize + 8;
                    }
                }
                size
            }
        }
        /// Estimated IMU deterministic errors. The mechanization removes these from
        /// every raw sample: corrected = (raw - bias*dt) / (1 + scale).
        #[derive(Debug, Default, Clone, Copy)]
        pub struct ImuError {
            /// Gyroscope bias, rad/s.
            ///
            /// *Note:* The presence of this field is tracked separately in the `_has` field. It's recommended to access this field via the accessor rather than directly.
            pub r#gyro_bias_rps: Vec3,
            /// Accelerometer bias, m/s^2.
            ///
            /// *Note:* The presence of this field is tracked separately in the `_has` field. It's recommended to access this field via the accessor rather than directly.
            pub r#accel_bias_mps2: Vec3,
            /// Gyroscope scale-factor error, dimensionless (1e-6 is 1 ppm).
            ///
            /// *Note:* The presence of this field is tracked separately in the `_has` field. It's recommended to access this field via the accessor rather than directly.
            pub r#gyro_scale: Vec3,
            /// Accelerometer scale-factor error, dimensionless.
            ///
            /// *Note:* The presence of this field is tracked separately in the `_has` field. It's recommended to access this field via the accessor rather than directly.
            pub r#accel_scale: Vec3,
            /// Tracks presence of optional and message fields
            pub _has: ImuError_::_Hazzer,
        }
        impl ::core::cmp::PartialEq for ImuError {
            fn eq(&self, other: &Self) -> bool {
                let mut ret = true;
                ret &= (self.r#gyro_bias_rps() == other.r#gyro_bias_rps());
                ret &= (self.r#accel_bias_mps2() == other.r#accel_bias_mps2());
                ret &= (self.r#gyro_scale() == other.r#gyro_scale());
                ret &= (self.r#accel_scale() == other.r#accel_scale());
                ret
            }
        }
        impl ImuError {
            /// Return a reference to `gyro_bias_rps` as an `Option`
            #[inline]
            pub fn r#gyro_bias_rps(&self) -> ::core::option::Option<&Vec3> {
                self._has.r#gyro_bias_rps().then_some(&self.r#gyro_bias_rps)
            }
            /// Set the value and presence of `gyro_bias_rps`
            #[inline]
            pub fn set_gyro_bias_rps(&mut self, value: Vec3) -> &mut Self {
                self._has.set_gyro_bias_rps();
                self.r#gyro_bias_rps = value.into();
                self
            }
            /// Return a mutable reference to `gyro_bias_rps` as an `Option`
            #[inline]
            pub fn mut_gyro_bias_rps(&mut self) -> ::core::option::Option<&mut Vec3> {
                self._has.r#gyro_bias_rps().then_some(&mut self.r#gyro_bias_rps)
            }
            /// Clear the presence of `gyro_bias_rps`
            #[inline]
            pub fn clear_gyro_bias_rps(&mut self) -> &mut Self {
                self._has.clear_gyro_bias_rps();
                self
            }
            /// Take the value of `gyro_bias_rps` and clear its presence
            #[inline]
            pub fn take_gyro_bias_rps(&mut self) -> ::core::option::Option<Vec3> {
                let val = self
                    ._has
                    .r#gyro_bias_rps()
                    .then(|| ::core::mem::take(&mut self.r#gyro_bias_rps));
                self._has.clear_gyro_bias_rps();
                val
            }
            /// Builder method that sets the value of `gyro_bias_rps`. Useful for initializing the message.
            #[inline]
            pub fn init_gyro_bias_rps(mut self, value: Vec3) -> Self {
                self.set_gyro_bias_rps(value);
                self
            }
            /// Return a reference to `accel_bias_mps2` as an `Option`
            #[inline]
            pub fn r#accel_bias_mps2(&self) -> ::core::option::Option<&Vec3> {
                self._has.r#accel_bias_mps2().then_some(&self.r#accel_bias_mps2)
            }
            /// Set the value and presence of `accel_bias_mps2`
            #[inline]
            pub fn set_accel_bias_mps2(&mut self, value: Vec3) -> &mut Self {
                self._has.set_accel_bias_mps2();
                self.r#accel_bias_mps2 = value.into();
                self
            }
            /// Return a mutable reference to `accel_bias_mps2` as an `Option`
            #[inline]
            pub fn mut_accel_bias_mps2(&mut self) -> ::core::option::Option<&mut Vec3> {
                self._has.r#accel_bias_mps2().then_some(&mut self.r#accel_bias_mps2)
            }
            /// Clear the presence of `accel_bias_mps2`
            #[inline]
            pub fn clear_accel_bias_mps2(&mut self) -> &mut Self {
                self._has.clear_accel_bias_mps2();
                self
            }
            /// Take the value of `accel_bias_mps2` and clear its presence
            #[inline]
            pub fn take_accel_bias_mps2(&mut self) -> ::core::option::Option<Vec3> {
                let val = self
                    ._has
                    .r#accel_bias_mps2()
                    .then(|| ::core::mem::take(&mut self.r#accel_bias_mps2));
                self._has.clear_accel_bias_mps2();
                val
            }
            /// Builder method that sets the value of `accel_bias_mps2`. Useful for initializing the message.
            #[inline]
            pub fn init_accel_bias_mps2(mut self, value: Vec3) -> Self {
                self.set_accel_bias_mps2(value);
                self
            }
            /// Return a reference to `gyro_scale` as an `Option`
            #[inline]
            pub fn r#gyro_scale(&self) -> ::core::option::Option<&Vec3> {
                self._has.r#gyro_scale().then_some(&self.r#gyro_scale)
            }
            /// Set the value and presence of `gyro_scale`
            #[inline]
            pub fn set_gyro_scale(&mut self, value: Vec3) -> &mut Self {
                self._has.set_gyro_scale();
                self.r#gyro_scale = value.into();
                self
            }
            /// Return a mutable reference to `gyro_scale` as an `Option`
            #[inline]
            pub fn mut_gyro_scale(&mut self) -> ::core::option::Option<&mut Vec3> {
                self._has.r#gyro_scale().then_some(&mut self.r#gyro_scale)
            }
            /// Clear the presence of `gyro_scale`
            #[inline]
            pub fn clear_gyro_scale(&mut self) -> &mut Self {
                self._has.clear_gyro_scale();
                self
            }
            /// Take the value of `gyro_scale` and clear its presence
            #[inline]
            pub fn take_gyro_scale(&mut self) -> ::core::option::Option<Vec3> {
                let val = self
                    ._has
                    .r#gyro_scale()
                    .then(|| ::core::mem::take(&mut self.r#gyro_scale));
                self._has.clear_gyro_scale();
                val
            }
            /// Builder method that sets the value of `gyro_scale`. Useful for initializing the message.
            #[inline]
            pub fn init_gyro_scale(mut self, value: Vec3) -> Self {
                self.set_gyro_scale(value);
                self
            }
            /// Return a reference to `accel_scale` as an `Option`
            #[inline]
            pub fn r#accel_scale(&self) -> ::core::option::Option<&Vec3> {
                self._has.r#accel_scale().then_some(&self.r#accel_scale)
            }
            /// Set the value and presence of `accel_scale`
            #[inline]
            pub fn set_accel_scale(&mut self, value: Vec3) -> &mut Self {
                self._has.set_accel_scale();
                self.r#accel_scale = value.into();
                self
            }
            /// Return a mutable reference to `accel_scale` as an `Option`
            #[inline]
            pub fn mut_accel_scale(&mut self) -> ::core::option::Option<&mut Vec3> {
                self._has.r#accel_scale().then_some(&mut self.r#accel_scale)
            }
            /// Clear the presence of `accel_scale`
            #[inline]
            pub fn clear_accel_scale(&mut self) -> &mut Self {
                self._has.clear_accel_scale();
                self
            }
            /// Take the value of `accel_scale` and clear its presence
            #[inline]
            pub fn take_accel_scale(&mut self) -> ::core::option::Option<Vec3> {
                let val = self
                    ._has
                    .r#accel_scale()
                    .then(|| ::core::mem::take(&mut self.r#accel_scale));
                self._has.clear_accel_scale();
                val
            }
            /// Builder method that sets the value of `accel_scale`. Useful for initializing the message.
            #[inline]
            pub fn init_accel_scale(mut self, value: Vec3) -> Self {
                self.set_accel_scale(value);
                self
            }
        }
        impl ::micropb::MessageDecode for ImuError {
            fn decode<IMPL_MICROPB_READ: ::micropb::PbRead>(
                &mut self,
                decoder: &mut ::micropb::PbDecoder<IMPL_MICROPB_READ>,
                len: usize,
            ) -> Result<(), ::micropb::DecodeError<IMPL_MICROPB_READ::Error>> {
                use ::micropb::{PbBytes, PbString, PbVec, PbMap, FieldDecode};
                let before = decoder.bytes_read();
                while decoder.bytes_read() - before < len {
                    let tag = decoder.decode_tag()?;
                    match tag.field_num() {
                        0 => return Err(::micropb::DecodeError::ZeroField),
                        1u32 => {
                            let mut_ref = &mut self.r#gyro_bias_rps;
                            {
                                mut_ref.decode_len_delimited(decoder)?;
                            };
                            self._has.set_gyro_bias_rps();
                        }
                        2u32 => {
                            let mut_ref = &mut self.r#accel_bias_mps2;
                            {
                                mut_ref.decode_len_delimited(decoder)?;
                            };
                            self._has.set_accel_bias_mps2();
                        }
                        3u32 => {
                            let mut_ref = &mut self.r#gyro_scale;
                            {
                                mut_ref.decode_len_delimited(decoder)?;
                            };
                            self._has.set_gyro_scale();
                        }
                        4u32 => {
                            let mut_ref = &mut self.r#accel_scale;
                            {
                                mut_ref.decode_len_delimited(decoder)?;
                            };
                            self._has.set_accel_scale();
                        }
                        _ => {
                            decoder.skip_wire_value(tag.wire_type())?;
                        }
                    }
                }
                Ok(())
            }
        }
        impl ::micropb::MessageEncode for ImuError {
            const MAX_SIZE: ::core::result::Result<usize, &'static str> = 'msg: {
                let mut max_size = 0;
                match ::micropb::const_map!(
                    ::micropb::const_map!(< Vec3 as ::micropb::MessageEncode >
                    ::MAX_SIZE, | size | ::micropb::size::sizeof_len_record(size)), |
                    size | size + 1usize
                ) {
                    ::core::result::Result::Ok(size) => {
                        max_size += size;
                    }
                    ::core::result::Result::Err(err) => {
                        break 'msg (::core::result::Result::<usize, _>::Err(err));
                    }
                }
                match ::micropb::const_map!(
                    ::micropb::const_map!(< Vec3 as ::micropb::MessageEncode >
                    ::MAX_SIZE, | size | ::micropb::size::sizeof_len_record(size)), |
                    size | size + 1usize
                ) {
                    ::core::result::Result::Ok(size) => {
                        max_size += size;
                    }
                    ::core::result::Result::Err(err) => {
                        break 'msg (::core::result::Result::<usize, _>::Err(err));
                    }
                }
                match ::micropb::const_map!(
                    ::micropb::const_map!(< Vec3 as ::micropb::MessageEncode >
                    ::MAX_SIZE, | size | ::micropb::size::sizeof_len_record(size)), |
                    size | size + 1usize
                ) {
                    ::core::result::Result::Ok(size) => {
                        max_size += size;
                    }
                    ::core::result::Result::Err(err) => {
                        break 'msg (::core::result::Result::<usize, _>::Err(err));
                    }
                }
                match ::micropb::const_map!(
                    ::micropb::const_map!(< Vec3 as ::micropb::MessageEncode >
                    ::MAX_SIZE, | size | ::micropb::size::sizeof_len_record(size)), |
                    size | size + 1usize
                ) {
                    ::core::result::Result::Ok(size) => {
                        max_size += size;
                    }
                    ::core::result::Result::Err(err) => {
                        break 'msg (::core::result::Result::<usize, _>::Err(err));
                    }
                }
                ::core::result::Result::Ok(max_size)
            };
            fn encode<IMPL_MICROPB_WRITE: ::micropb::PbWrite>(
                &self,
                encoder: &mut ::micropb::PbEncoder<IMPL_MICROPB_WRITE>,
            ) -> Result<(), IMPL_MICROPB_WRITE::Error> {
                use ::micropb::{PbMap, FieldEncode};
                {
                    if let ::core::option::Option::Some(val_ref) = self.r#gyro_bias_rps()
                    {
                        encoder.encode_varint32(10u32)?;
                        val_ref.encode_len_delimited(encoder)?;
                    }
                }
                {
                    if let ::core::option::Option::Some(val_ref) = self
                        .r#accel_bias_mps2()
                    {
                        encoder.encode_varint32(18u32)?;
                        val_ref.encode_len_delimited(encoder)?;
                    }
                }
                {
                    if let ::core::option::Option::Some(val_ref) = self.r#gyro_scale() {
                        encoder.encode_varint32(26u32)?;
                        val_ref.encode_len_delimited(encoder)?;
                    }
                }
                {
                    if let ::core::option::Option::Some(val_ref) = self.r#accel_scale() {
                        encoder.encode_varint32(34u32)?;
                        val_ref.encode_len_delimited(encoder)?;
                    }
                }
                Ok(())
            }
            fn compute_size(&self) -> usize {
                use ::micropb::{PbMap, FieldEncode};
                let mut size = 0;
                {
                    if let ::core::option::Option::Some(val_ref) = self.r#gyro_bias_rps()
                    {
                        size
                            += 1usize
                                + ::micropb::size::sizeof_len_record(
                                    val_ref.compute_size(),
                                );
                    }
                }
                {
                    if let ::core::option::Option::Some(val_ref) = self
                        .r#accel_bias_mps2()
                    {
                        size
                            += 1usize
                                + ::micropb::size::sizeof_len_record(
                                    val_ref.compute_size(),
                                );
                    }
                }
                {
                    if let ::core::option::Option::Some(val_ref) = self.r#gyro_scale() {
                        size
                            += 1usize
                                + ::micropb::size::sizeof_len_record(
                                    val_ref.compute_size(),
                                );
                    }
                }
                {
                    if let ::core::option::Option::Some(val_ref) = self.r#accel_scale() {
                        size
                            += 1usize
                                + ::micropb::size::sizeof_len_record(
                                    val_ref.compute_size(),
                                );
                    }
                }
                size
            }
        }
        /// Inner types for `ImuError`
        pub mod ImuError_ {
            /// Compact bitfield for tracking presence of optional and message fields
            #[derive(Debug, Default, PartialEq, Clone, Copy)]
            pub struct _Hazzer([u8; 1]);
            impl _Hazzer {
                /// New hazzer with all fields set to off
                #[inline]
                pub const fn _new() -> Self {
                    Self([0; 1])
                }
                /// Query presence of `gyro_bias_rps`
                #[inline]
                pub const fn r#gyro_bias_rps(&self) -> bool {
                    (self.0[0] & 1) != 0
                }
                /// Set presence of `gyro_bias_rps`
                #[inline]
                pub const fn set_gyro_bias_rps(&mut self) -> &mut Self {
                    let elem = &mut self.0[0];
                    *elem |= 1;
                    self
                }
                /// Clear presence of `gyro_bias_rps`
                #[inline]
                pub const fn clear_gyro_bias_rps(&mut self) -> &mut Self {
                    let elem = &mut self.0[0];
                    *elem &= !1;
                    self
                }
                /// Builder method that sets the presence of `gyro_bias_rps`. Useful for initializing the Hazzer.
                #[inline]
                pub const fn init_gyro_bias_rps(mut self) -> Self {
                    self.set_gyro_bias_rps();
                    self
                }
                /// Query presence of `accel_bias_mps2`
                #[inline]
                pub const fn r#accel_bias_mps2(&self) -> bool {
                    (self.0[0] & 2) != 0
                }
                /// Set presence of `accel_bias_mps2`
                #[inline]
                pub const fn set_accel_bias_mps2(&mut self) -> &mut Self {
                    let elem = &mut self.0[0];
                    *elem |= 2;
                    self
                }
                /// Clear presence of `accel_bias_mps2`
                #[inline]
                pub const fn clear_accel_bias_mps2(&mut self) -> &mut Self {
                    let elem = &mut self.0[0];
                    *elem &= !2;
                    self
                }
                /// Builder method that sets the presence of `accel_bias_mps2`. Useful for initializing the Hazzer.
                #[inline]
                pub const fn init_accel_bias_mps2(mut self) -> Self {
                    self.set_accel_bias_mps2();
                    self
                }
                /// Query presence of `gyro_scale`
                #[inline]
                pub const fn r#gyro_scale(&self) -> bool {
                    (self.0[0] & 4) != 0
                }
                /// Set presence of `gyro_scale`
                #[inline]
                pub const fn set_gyro_scale(&mut self) -> &mut Self {
                    let elem = &mut self.0[0];
                    *elem |= 4;
                    self
                }
                /// Clear presence of `gyro_scale`
                #[inline]
                pub const fn clear_gyro_scale(&mut self) -> &mut Self {
                    let elem = &mut self.0[0];
                    *elem &= !4;
                    self
                }
                /// Builder method that sets the presence of `gyro_scale`. Useful for initializing the Hazzer.
                #[inline]
                pub const fn init_gyro_scale(mut self) -> Self {
                    self.set_gyro_scale();
                    self
                }
                /// Query presence of `accel_scale`
                #[inline]
                pub const fn r#accel_scale(&self) -> bool {
                    (self.0[0] & 8) != 0
                }
                /// Set presence of `accel_scale`
                #[inline]
                pub const fn set_accel_scale(&mut self) -> &mut Self {
                    let elem = &mut self.0[0];
                    *elem |= 8;
                    self
                }
                /// Clear presence of `accel_scale`
                #[inline]
                pub const fn clear_accel_scale(&mut self) -> &mut Self {
                    let elem = &mut self.0[0];
                    *elem &= !8;
                    self
                }
                /// Builder method that sets the presence of `accel_scale`. Useful for initializing the Hazzer.
                #[inline]
                pub const fn init_accel_scale(mut self) -> Self {
                    self.set_accel_scale();
                    self
                }
            }
        }
        /// A navigation solution at one epoch.
        #[derive(Debug, Default, Clone)]
        pub struct NavSolution {
            /// *Note:* The presence of this field is tracked separately in the `_has` field. It's recommended to access this field via the accessor rather than directly.
            pub r#time: GpsTime,
            /// Position of the IMU reference point, not the GNSS antenna.
            ///
            /// *Note:* The presence of this field is tracked separately in the `_has` field. It's recommended to access this field via the accessor rather than directly.
            pub r#position: Lla,
            /// Ground velocity in the NED navigation frame, m/s.
            ///
            /// *Note:* The presence of this field is tracked separately in the `_has` field. It's recommended to access this field via the accessor rather than directly.
            pub r#velocity_mps: Ned,
            /// Attitude q_nb. Authoritative.
            ///
            /// *Note:* The presence of this field is tracked separately in the `_has` field. It's recommended to access this field via the accessor rather than directly.
            pub r#attitude: Quaternion,
            /// Attitude as Euler angles. Derived from `attitude`, carried for the
            /// convenience of consumers that only plot; never round-trip through it.
            ///
            /// *Note:* The presence of this field is tracked separately in the `_has` field. It's recommended to access this field via the accessor rather than directly.
            pub r#euler: Euler,
            /// Estimated IMU errors at this epoch.
            ///
            /// *Note:* The presence of this field is tracked separately in the `_has` field. It's recommended to access this field via the accessor rather than directly.
            pub r#imu_error: ImuError,
            /// Per-state one-sigma uncertainties, 21 elements in the order given by
            /// drifters_filter::state: position, velocity, attitude, gyro bias, accel
            /// bias, gyro scale, accel scale.
            pub r#state_std: ::heapless::Vec<f64, 21>,
            /// Tracks presence of optional and message fields
            pub _has: NavSolution_::_Hazzer,
        }
        impl ::core::cmp::PartialEq for NavSolution {
            fn eq(&self, other: &Self) -> bool {
                let mut ret = true;
                ret &= (self.r#time() == other.r#time());
                ret &= (self.r#position() == other.r#position());
                ret &= (self.r#velocity_mps() == other.r#velocity_mps());
                ret &= (self.r#attitude() == other.r#attitude());
                ret &= (self.r#euler() == other.r#euler());
                ret &= (self.r#imu_error() == other.r#imu_error());
                ret &= (self.r#state_std == other.r#state_std);
                ret
            }
        }
        impl NavSolution {
            /// Return a reference to `time` as an `Option`
            #[inline]
            pub fn r#time(&self) -> ::core::option::Option<&GpsTime> {
                self._has.r#time().then_some(&self.r#time)
            }
            /// Set the value and presence of `time`
            #[inline]
            pub fn set_time(&mut self, value: GpsTime) -> &mut Self {
                self._has.set_time();
                self.r#time = value.into();
                self
            }
            /// Return a mutable reference to `time` as an `Option`
            #[inline]
            pub fn mut_time(&mut self) -> ::core::option::Option<&mut GpsTime> {
                self._has.r#time().then_some(&mut self.r#time)
            }
            /// Clear the presence of `time`
            #[inline]
            pub fn clear_time(&mut self) -> &mut Self {
                self._has.clear_time();
                self
            }
            /// Take the value of `time` and clear its presence
            #[inline]
            pub fn take_time(&mut self) -> ::core::option::Option<GpsTime> {
                let val = self
                    ._has
                    .r#time()
                    .then(|| ::core::mem::take(&mut self.r#time));
                self._has.clear_time();
                val
            }
            /// Builder method that sets the value of `time`. Useful for initializing the message.
            #[inline]
            pub fn init_time(mut self, value: GpsTime) -> Self {
                self.set_time(value);
                self
            }
            /// Return a reference to `position` as an `Option`
            #[inline]
            pub fn r#position(&self) -> ::core::option::Option<&Lla> {
                self._has.r#position().then_some(&self.r#position)
            }
            /// Set the value and presence of `position`
            #[inline]
            pub fn set_position(&mut self, value: Lla) -> &mut Self {
                self._has.set_position();
                self.r#position = value.into();
                self
            }
            /// Return a mutable reference to `position` as an `Option`
            #[inline]
            pub fn mut_position(&mut self) -> ::core::option::Option<&mut Lla> {
                self._has.r#position().then_some(&mut self.r#position)
            }
            /// Clear the presence of `position`
            #[inline]
            pub fn clear_position(&mut self) -> &mut Self {
                self._has.clear_position();
                self
            }
            /// Take the value of `position` and clear its presence
            #[inline]
            pub fn take_position(&mut self) -> ::core::option::Option<Lla> {
                let val = self
                    ._has
                    .r#position()
                    .then(|| ::core::mem::take(&mut self.r#position));
                self._has.clear_position();
                val
            }
            /// Builder method that sets the value of `position`. Useful for initializing the message.
            #[inline]
            pub fn init_position(mut self, value: Lla) -> Self {
                self.set_position(value);
                self
            }
            /// Return a reference to `velocity_mps` as an `Option`
            #[inline]
            pub fn r#velocity_mps(&self) -> ::core::option::Option<&Ned> {
                self._has.r#velocity_mps().then_some(&self.r#velocity_mps)
            }
            /// Set the value and presence of `velocity_mps`
            #[inline]
            pub fn set_velocity_mps(&mut self, value: Ned) -> &mut Self {
                self._has.set_velocity_mps();
                self.r#velocity_mps = value.into();
                self
            }
            /// Return a mutable reference to `velocity_mps` as an `Option`
            #[inline]
            pub fn mut_velocity_mps(&mut self) -> ::core::option::Option<&mut Ned> {
                self._has.r#velocity_mps().then_some(&mut self.r#velocity_mps)
            }
            /// Clear the presence of `velocity_mps`
            #[inline]
            pub fn clear_velocity_mps(&mut self) -> &mut Self {
                self._has.clear_velocity_mps();
                self
            }
            /// Take the value of `velocity_mps` and clear its presence
            #[inline]
            pub fn take_velocity_mps(&mut self) -> ::core::option::Option<Ned> {
                let val = self
                    ._has
                    .r#velocity_mps()
                    .then(|| ::core::mem::take(&mut self.r#velocity_mps));
                self._has.clear_velocity_mps();
                val
            }
            /// Builder method that sets the value of `velocity_mps`. Useful for initializing the message.
            #[inline]
            pub fn init_velocity_mps(mut self, value: Ned) -> Self {
                self.set_velocity_mps(value);
                self
            }
            /// Return a reference to `attitude` as an `Option`
            #[inline]
            pub fn r#attitude(&self) -> ::core::option::Option<&Quaternion> {
                self._has.r#attitude().then_some(&self.r#attitude)
            }
            /// Set the value and presence of `attitude`
            #[inline]
            pub fn set_attitude(&mut self, value: Quaternion) -> &mut Self {
                self._has.set_attitude();
                self.r#attitude = value.into();
                self
            }
            /// Return a mutable reference to `attitude` as an `Option`
            #[inline]
            pub fn mut_attitude(&mut self) -> ::core::option::Option<&mut Quaternion> {
                self._has.r#attitude().then_some(&mut self.r#attitude)
            }
            /// Clear the presence of `attitude`
            #[inline]
            pub fn clear_attitude(&mut self) -> &mut Self {
                self._has.clear_attitude();
                self
            }
            /// Take the value of `attitude` and clear its presence
            #[inline]
            pub fn take_attitude(&mut self) -> ::core::option::Option<Quaternion> {
                let val = self
                    ._has
                    .r#attitude()
                    .then(|| ::core::mem::take(&mut self.r#attitude));
                self._has.clear_attitude();
                val
            }
            /// Builder method that sets the value of `attitude`. Useful for initializing the message.
            #[inline]
            pub fn init_attitude(mut self, value: Quaternion) -> Self {
                self.set_attitude(value);
                self
            }
            /// Return a reference to `euler` as an `Option`
            #[inline]
            pub fn r#euler(&self) -> ::core::option::Option<&Euler> {
                self._has.r#euler().then_some(&self.r#euler)
            }
            /// Set the value and presence of `euler`
            #[inline]
            pub fn set_euler(&mut self, value: Euler) -> &mut Self {
                self._has.set_euler();
                self.r#euler = value.into();
                self
            }
            /// Return a mutable reference to `euler` as an `Option`
            #[inline]
            pub fn mut_euler(&mut self) -> ::core::option::Option<&mut Euler> {
                self._has.r#euler().then_some(&mut self.r#euler)
            }
            /// Clear the presence of `euler`
            #[inline]
            pub fn clear_euler(&mut self) -> &mut Self {
                self._has.clear_euler();
                self
            }
            /// Take the value of `euler` and clear its presence
            #[inline]
            pub fn take_euler(&mut self) -> ::core::option::Option<Euler> {
                let val = self
                    ._has
                    .r#euler()
                    .then(|| ::core::mem::take(&mut self.r#euler));
                self._has.clear_euler();
                val
            }
            /// Builder method that sets the value of `euler`. Useful for initializing the message.
            #[inline]
            pub fn init_euler(mut self, value: Euler) -> Self {
                self.set_euler(value);
                self
            }
            /// Return a reference to `imu_error` as an `Option`
            #[inline]
            pub fn r#imu_error(&self) -> ::core::option::Option<&ImuError> {
                self._has.r#imu_error().then_some(&self.r#imu_error)
            }
            /// Set the value and presence of `imu_error`
            #[inline]
            pub fn set_imu_error(&mut self, value: ImuError) -> &mut Self {
                self._has.set_imu_error();
                self.r#imu_error = value.into();
                self
            }
            /// Return a mutable reference to `imu_error` as an `Option`
            #[inline]
            pub fn mut_imu_error(&mut self) -> ::core::option::Option<&mut ImuError> {
                self._has.r#imu_error().then_some(&mut self.r#imu_error)
            }
            /// Clear the presence of `imu_error`
            #[inline]
            pub fn clear_imu_error(&mut self) -> &mut Self {
                self._has.clear_imu_error();
                self
            }
            /// Take the value of `imu_error` and clear its presence
            #[inline]
            pub fn take_imu_error(&mut self) -> ::core::option::Option<ImuError> {
                let val = self
                    ._has
                    .r#imu_error()
                    .then(|| ::core::mem::take(&mut self.r#imu_error));
                self._has.clear_imu_error();
                val
            }
            /// Builder method that sets the value of `imu_error`. Useful for initializing the message.
            #[inline]
            pub fn init_imu_error(mut self, value: ImuError) -> Self {
                self.set_imu_error(value);
                self
            }
        }
        impl ::micropb::MessageDecode for NavSolution {
            fn decode<IMPL_MICROPB_READ: ::micropb::PbRead>(
                &mut self,
                decoder: &mut ::micropb::PbDecoder<IMPL_MICROPB_READ>,
                len: usize,
            ) -> Result<(), ::micropb::DecodeError<IMPL_MICROPB_READ::Error>> {
                use ::micropb::{PbBytes, PbString, PbVec, PbMap, FieldDecode};
                let before = decoder.bytes_read();
                while decoder.bytes_read() - before < len {
                    let tag = decoder.decode_tag()?;
                    match tag.field_num() {
                        0 => return Err(::micropb::DecodeError::ZeroField),
                        1u32 => {
                            let mut_ref = &mut self.r#time;
                            {
                                mut_ref.decode_len_delimited(decoder)?;
                            };
                            self._has.set_time();
                        }
                        2u32 => {
                            let mut_ref = &mut self.r#position;
                            {
                                mut_ref.decode_len_delimited(decoder)?;
                            };
                            self._has.set_position();
                        }
                        3u32 => {
                            let mut_ref = &mut self.r#velocity_mps;
                            {
                                mut_ref.decode_len_delimited(decoder)?;
                            };
                            self._has.set_velocity_mps();
                        }
                        4u32 => {
                            let mut_ref = &mut self.r#attitude;
                            {
                                mut_ref.decode_len_delimited(decoder)?;
                            };
                            self._has.set_attitude();
                        }
                        5u32 => {
                            let mut_ref = &mut self.r#euler;
                            {
                                mut_ref.decode_len_delimited(decoder)?;
                            };
                            self._has.set_euler();
                        }
                        6u32 => {
                            let mut_ref = &mut self.r#imu_error;
                            {
                                mut_ref.decode_len_delimited(decoder)?;
                            };
                            self._has.set_imu_error();
                        }
                        7u32 => {
                            if tag.wire_type() == ::micropb::WIRE_TYPE_LEN {
                                decoder
                                    .decode_packed(
                                        &mut self.r#state_std,
                                        |decoder| decoder.decode_double().map(|v| v as _),
                                    )?;
                            } else {
                                if let (Err(_), false) = (
                                    self.r#state_std.pb_push(decoder.decode_double()? as _),
                                    decoder.ignore_repeated_cap_err,
                                ) {
                                    return Err(::micropb::DecodeError::Capacity);
                                }
                            }
                        }
                        _ => {
                            decoder.skip_wire_value(tag.wire_type())?;
                        }
                    }
                }
                Ok(())
            }
        }
        impl ::micropb::MessageEncode for NavSolution {
            const MAX_SIZE: ::core::result::Result<usize, &'static str> = 'msg: {
                let mut max_size = 0;
                match ::micropb::const_map!(
                    ::micropb::const_map!(< GpsTime as ::micropb::MessageEncode >
                    ::MAX_SIZE, | size | ::micropb::size::sizeof_len_record(size)), |
                    size | size + 1usize
                ) {
                    ::core::result::Result::Ok(size) => {
                        max_size += size;
                    }
                    ::core::result::Result::Err(err) => {
                        break 'msg (::core::result::Result::<usize, _>::Err(err));
                    }
                }
                match ::micropb::const_map!(
                    ::micropb::const_map!(< Lla as ::micropb::MessageEncode > ::MAX_SIZE,
                    | size | ::micropb::size::sizeof_len_record(size)), | size | size +
                    1usize
                ) {
                    ::core::result::Result::Ok(size) => {
                        max_size += size;
                    }
                    ::core::result::Result::Err(err) => {
                        break 'msg (::core::result::Result::<usize, _>::Err(err));
                    }
                }
                match ::micropb::const_map!(
                    ::micropb::const_map!(< Ned as ::micropb::MessageEncode > ::MAX_SIZE,
                    | size | ::micropb::size::sizeof_len_record(size)), | size | size +
                    1usize
                ) {
                    ::core::result::Result::Ok(size) => {
                        max_size += size;
                    }
                    ::core::result::Result::Err(err) => {
                        break 'msg (::core::result::Result::<usize, _>::Err(err));
                    }
                }
                match ::micropb::const_map!(
                    ::micropb::const_map!(< Quaternion as ::micropb::MessageEncode >
                    ::MAX_SIZE, | size | ::micropb::size::sizeof_len_record(size)), |
                    size | size + 1usize
                ) {
                    ::core::result::Result::Ok(size) => {
                        max_size += size;
                    }
                    ::core::result::Result::Err(err) => {
                        break 'msg (::core::result::Result::<usize, _>::Err(err));
                    }
                }
                match ::micropb::const_map!(
                    ::micropb::const_map!(< Euler as ::micropb::MessageEncode >
                    ::MAX_SIZE, | size | ::micropb::size::sizeof_len_record(size)), |
                    size | size + 1usize
                ) {
                    ::core::result::Result::Ok(size) => {
                        max_size += size;
                    }
                    ::core::result::Result::Err(err) => {
                        break 'msg (::core::result::Result::<usize, _>::Err(err));
                    }
                }
                match ::micropb::const_map!(
                    ::micropb::const_map!(< ImuError as ::micropb::MessageEncode >
                    ::MAX_SIZE, | size | ::micropb::size::sizeof_len_record(size)), |
                    size | size + 1usize
                ) {
                    ::core::result::Result::Ok(size) => {
                        max_size += size;
                    }
                    ::core::result::Result::Err(err) => {
                        break 'msg (::core::result::Result::<usize, _>::Err(err));
                    }
                }
                match ::micropb::const_map!(
                    ::core::result::Result::Ok(8usize), | size |
                    ::micropb::size::sizeof_len_record(21usize * size) + 1usize
                ) {
                    ::core::result::Result::Ok(size) => {
                        max_size += size;
                    }
                    ::core::result::Result::Err(err) => {
                        break 'msg (::core::result::Result::<usize, _>::Err(err));
                    }
                }
                ::core::result::Result::Ok(max_size)
            };
            fn encode<IMPL_MICROPB_WRITE: ::micropb::PbWrite>(
                &self,
                encoder: &mut ::micropb::PbEncoder<IMPL_MICROPB_WRITE>,
            ) -> Result<(), IMPL_MICROPB_WRITE::Error> {
                use ::micropb::{PbMap, FieldEncode};
                {
                    if let ::core::option::Option::Some(val_ref) = self.r#time() {
                        encoder.encode_varint32(10u32)?;
                        val_ref.encode_len_delimited(encoder)?;
                    }
                }
                {
                    if let ::core::option::Option::Some(val_ref) = self.r#position() {
                        encoder.encode_varint32(18u32)?;
                        val_ref.encode_len_delimited(encoder)?;
                    }
                }
                {
                    if let ::core::option::Option::Some(val_ref) = self.r#velocity_mps()
                    {
                        encoder.encode_varint32(26u32)?;
                        val_ref.encode_len_delimited(encoder)?;
                    }
                }
                {
                    if let ::core::option::Option::Some(val_ref) = self.r#attitude() {
                        encoder.encode_varint32(34u32)?;
                        val_ref.encode_len_delimited(encoder)?;
                    }
                }
                {
                    if let ::core::option::Option::Some(val_ref) = self.r#euler() {
                        encoder.encode_varint32(42u32)?;
                        val_ref.encode_len_delimited(encoder)?;
                    }
                }
                {
                    if let ::core::option::Option::Some(val_ref) = self.r#imu_error() {
                        encoder.encode_varint32(50u32)?;
                        val_ref.encode_len_delimited(encoder)?;
                    }
                }
                {
                    if !self.r#state_std.is_empty() {
                        let len = self.r#state_std.len() * 8usize;
                        encoder.encode_varint32(58u32)?;
                        encoder
                            .encode_packed(
                                len,
                                &self.r#state_std,
                                |encoder, val| {
                                    let val_ref = &val;
                                    encoder.encode_double(*val_ref)
                                },
                            )?;
                    }
                }
                Ok(())
            }
            fn compute_size(&self) -> usize {
                use ::micropb::{PbMap, FieldEncode};
                let mut size = 0;
                {
                    if let ::core::option::Option::Some(val_ref) = self.r#time() {
                        size
                            += 1usize
                                + ::micropb::size::sizeof_len_record(
                                    val_ref.compute_size(),
                                );
                    }
                }
                {
                    if let ::core::option::Option::Some(val_ref) = self.r#position() {
                        size
                            += 1usize
                                + ::micropb::size::sizeof_len_record(
                                    val_ref.compute_size(),
                                );
                    }
                }
                {
                    if let ::core::option::Option::Some(val_ref) = self.r#velocity_mps()
                    {
                        size
                            += 1usize
                                + ::micropb::size::sizeof_len_record(
                                    val_ref.compute_size(),
                                );
                    }
                }
                {
                    if let ::core::option::Option::Some(val_ref) = self.r#attitude() {
                        size
                            += 1usize
                                + ::micropb::size::sizeof_len_record(
                                    val_ref.compute_size(),
                                );
                    }
                }
                {
                    if let ::core::option::Option::Some(val_ref) = self.r#euler() {
                        size
                            += 1usize
                                + ::micropb::size::sizeof_len_record(
                                    val_ref.compute_size(),
                                );
                    }
                }
                {
                    if let ::core::option::Option::Some(val_ref) = self.r#imu_error() {
                        size
                            += 1usize
                                + ::micropb::size::sizeof_len_record(
                                    val_ref.compute_size(),
                                );
                    }
                }
                {
                    if !self.r#state_std.is_empty() {
                        let len = self.r#state_std.len() * 8usize;
                        size += 1usize + ::micropb::size::sizeof_len_record(len);
                    }
                }
                size
            }
        }
        /// Inner types for `NavSolution`
        pub mod NavSolution_ {
            /// Compact bitfield for tracking presence of optional and message fields
            #[derive(Debug, Default, PartialEq, Clone, Copy)]
            pub struct _Hazzer([u8; 1]);
            impl _Hazzer {
                /// New hazzer with all fields set to off
                #[inline]
                pub const fn _new() -> Self {
                    Self([0; 1])
                }
                /// Query presence of `time`
                #[inline]
                pub const fn r#time(&self) -> bool {
                    (self.0[0] & 1) != 0
                }
                /// Set presence of `time`
                #[inline]
                pub const fn set_time(&mut self) -> &mut Self {
                    let elem = &mut self.0[0];
                    *elem |= 1;
                    self
                }
                /// Clear presence of `time`
                #[inline]
                pub const fn clear_time(&mut self) -> &mut Self {
                    let elem = &mut self.0[0];
                    *elem &= !1;
                    self
                }
                /// Builder method that sets the presence of `time`. Useful for initializing the Hazzer.
                #[inline]
                pub const fn init_time(mut self) -> Self {
                    self.set_time();
                    self
                }
                /// Query presence of `position`
                #[inline]
                pub const fn r#position(&self) -> bool {
                    (self.0[0] & 2) != 0
                }
                /// Set presence of `position`
                #[inline]
                pub const fn set_position(&mut self) -> &mut Self {
                    let elem = &mut self.0[0];
                    *elem |= 2;
                    self
                }
                /// Clear presence of `position`
                #[inline]
                pub const fn clear_position(&mut self) -> &mut Self {
                    let elem = &mut self.0[0];
                    *elem &= !2;
                    self
                }
                /// Builder method that sets the presence of `position`. Useful for initializing the Hazzer.
                #[inline]
                pub const fn init_position(mut self) -> Self {
                    self.set_position();
                    self
                }
                /// Query presence of `velocity_mps`
                #[inline]
                pub const fn r#velocity_mps(&self) -> bool {
                    (self.0[0] & 4) != 0
                }
                /// Set presence of `velocity_mps`
                #[inline]
                pub const fn set_velocity_mps(&mut self) -> &mut Self {
                    let elem = &mut self.0[0];
                    *elem |= 4;
                    self
                }
                /// Clear presence of `velocity_mps`
                #[inline]
                pub const fn clear_velocity_mps(&mut self) -> &mut Self {
                    let elem = &mut self.0[0];
                    *elem &= !4;
                    self
                }
                /// Builder method that sets the presence of `velocity_mps`. Useful for initializing the Hazzer.
                #[inline]
                pub const fn init_velocity_mps(mut self) -> Self {
                    self.set_velocity_mps();
                    self
                }
                /// Query presence of `attitude`
                #[inline]
                pub const fn r#attitude(&self) -> bool {
                    (self.0[0] & 8) != 0
                }
                /// Set presence of `attitude`
                #[inline]
                pub const fn set_attitude(&mut self) -> &mut Self {
                    let elem = &mut self.0[0];
                    *elem |= 8;
                    self
                }
                /// Clear presence of `attitude`
                #[inline]
                pub const fn clear_attitude(&mut self) -> &mut Self {
                    let elem = &mut self.0[0];
                    *elem &= !8;
                    self
                }
                /// Builder method that sets the presence of `attitude`. Useful for initializing the Hazzer.
                #[inline]
                pub const fn init_attitude(mut self) -> Self {
                    self.set_attitude();
                    self
                }
                /// Query presence of `euler`
                #[inline]
                pub const fn r#euler(&self) -> bool {
                    (self.0[0] & 16) != 0
                }
                /// Set presence of `euler`
                #[inline]
                pub const fn set_euler(&mut self) -> &mut Self {
                    let elem = &mut self.0[0];
                    *elem |= 16;
                    self
                }
                /// Clear presence of `euler`
                #[inline]
                pub const fn clear_euler(&mut self) -> &mut Self {
                    let elem = &mut self.0[0];
                    *elem &= !16;
                    self
                }
                /// Builder method that sets the presence of `euler`. Useful for initializing the Hazzer.
                #[inline]
                pub const fn init_euler(mut self) -> Self {
                    self.set_euler();
                    self
                }
                /// Query presence of `imu_error`
                #[inline]
                pub const fn r#imu_error(&self) -> bool {
                    (self.0[0] & 32) != 0
                }
                /// Set presence of `imu_error`
                #[inline]
                pub const fn set_imu_error(&mut self) -> &mut Self {
                    let elem = &mut self.0[0];
                    *elem |= 32;
                    self
                }
                /// Clear presence of `imu_error`
                #[inline]
                pub const fn clear_imu_error(&mut self) -> &mut Self {
                    let elem = &mut self.0[0];
                    *elem &= !32;
                    self
                }
                /// Builder method that sets the presence of `imu_error`. Useful for initializing the Hazzer.
                #[inline]
                pub const fn init_imu_error(mut self) -> Self {
                    self.set_imu_error();
                    self
                }
            }
        }
        /// The full 21x21 error-state covariance, row-major, 441 elements.
        ///
        /// Separate from NavSolution because it is large and usually only wanted for
        /// diagnostics — logging it at IMU rate is rarely what anyone means to do.
        #[derive(Debug, Default, Clone)]
        pub struct Covariance {
            /// *Note:* The presence of this field is tracked separately in the `_has` field. It's recommended to access this field via the accessor rather than directly.
            pub r#time: GpsTime,
            pub r#row_major: ::heapless::Vec<f64, 441>,
            /// Tracks presence of optional and message fields
            pub _has: Covariance_::_Hazzer,
        }
        impl ::core::cmp::PartialEq for Covariance {
            fn eq(&self, other: &Self) -> bool {
                let mut ret = true;
                ret &= (self.r#time() == other.r#time());
                ret &= (self.r#row_major == other.r#row_major);
                ret
            }
        }
        impl Covariance {
            /// Return a reference to `time` as an `Option`
            #[inline]
            pub fn r#time(&self) -> ::core::option::Option<&GpsTime> {
                self._has.r#time().then_some(&self.r#time)
            }
            /// Set the value and presence of `time`
            #[inline]
            pub fn set_time(&mut self, value: GpsTime) -> &mut Self {
                self._has.set_time();
                self.r#time = value.into();
                self
            }
            /// Return a mutable reference to `time` as an `Option`
            #[inline]
            pub fn mut_time(&mut self) -> ::core::option::Option<&mut GpsTime> {
                self._has.r#time().then_some(&mut self.r#time)
            }
            /// Clear the presence of `time`
            #[inline]
            pub fn clear_time(&mut self) -> &mut Self {
                self._has.clear_time();
                self
            }
            /// Take the value of `time` and clear its presence
            #[inline]
            pub fn take_time(&mut self) -> ::core::option::Option<GpsTime> {
                let val = self
                    ._has
                    .r#time()
                    .then(|| ::core::mem::take(&mut self.r#time));
                self._has.clear_time();
                val
            }
            /// Builder method that sets the value of `time`. Useful for initializing the message.
            #[inline]
            pub fn init_time(mut self, value: GpsTime) -> Self {
                self.set_time(value);
                self
            }
        }
        impl ::micropb::MessageDecode for Covariance {
            fn decode<IMPL_MICROPB_READ: ::micropb::PbRead>(
                &mut self,
                decoder: &mut ::micropb::PbDecoder<IMPL_MICROPB_READ>,
                len: usize,
            ) -> Result<(), ::micropb::DecodeError<IMPL_MICROPB_READ::Error>> {
                use ::micropb::{PbBytes, PbString, PbVec, PbMap, FieldDecode};
                let before = decoder.bytes_read();
                while decoder.bytes_read() - before < len {
                    let tag = decoder.decode_tag()?;
                    match tag.field_num() {
                        0 => return Err(::micropb::DecodeError::ZeroField),
                        1u32 => {
                            let mut_ref = &mut self.r#time;
                            {
                                mut_ref.decode_len_delimited(decoder)?;
                            };
                            self._has.set_time();
                        }
                        2u32 => {
                            if tag.wire_type() == ::micropb::WIRE_TYPE_LEN {
                                decoder
                                    .decode_packed(
                                        &mut self.r#row_major,
                                        |decoder| decoder.decode_double().map(|v| v as _),
                                    )?;
                            } else {
                                if let (Err(_), false) = (
                                    self.r#row_major.pb_push(decoder.decode_double()? as _),
                                    decoder.ignore_repeated_cap_err,
                                ) {
                                    return Err(::micropb::DecodeError::Capacity);
                                }
                            }
                        }
                        _ => {
                            decoder.skip_wire_value(tag.wire_type())?;
                        }
                    }
                }
                Ok(())
            }
        }
        impl ::micropb::MessageEncode for Covariance {
            const MAX_SIZE: ::core::result::Result<usize, &'static str> = 'msg: {
                let mut max_size = 0;
                match ::micropb::const_map!(
                    ::micropb::const_map!(< GpsTime as ::micropb::MessageEncode >
                    ::MAX_SIZE, | size | ::micropb::size::sizeof_len_record(size)), |
                    size | size + 1usize
                ) {
                    ::core::result::Result::Ok(size) => {
                        max_size += size;
                    }
                    ::core::result::Result::Err(err) => {
                        break 'msg (::core::result::Result::<usize, _>::Err(err));
                    }
                }
                match ::micropb::const_map!(
                    ::core::result::Result::Ok(8usize), | size |
                    ::micropb::size::sizeof_len_record(441usize * size) + 1usize
                ) {
                    ::core::result::Result::Ok(size) => {
                        max_size += size;
                    }
                    ::core::result::Result::Err(err) => {
                        break 'msg (::core::result::Result::<usize, _>::Err(err));
                    }
                }
                ::core::result::Result::Ok(max_size)
            };
            fn encode<IMPL_MICROPB_WRITE: ::micropb::PbWrite>(
                &self,
                encoder: &mut ::micropb::PbEncoder<IMPL_MICROPB_WRITE>,
            ) -> Result<(), IMPL_MICROPB_WRITE::Error> {
                use ::micropb::{PbMap, FieldEncode};
                {
                    if let ::core::option::Option::Some(val_ref) = self.r#time() {
                        encoder.encode_varint32(10u32)?;
                        val_ref.encode_len_delimited(encoder)?;
                    }
                }
                {
                    if !self.r#row_major.is_empty() {
                        let len = self.r#row_major.len() * 8usize;
                        encoder.encode_varint32(18u32)?;
                        encoder
                            .encode_packed(
                                len,
                                &self.r#row_major,
                                |encoder, val| {
                                    let val_ref = &val;
                                    encoder.encode_double(*val_ref)
                                },
                            )?;
                    }
                }
                Ok(())
            }
            fn compute_size(&self) -> usize {
                use ::micropb::{PbMap, FieldEncode};
                let mut size = 0;
                {
                    if let ::core::option::Option::Some(val_ref) = self.r#time() {
                        size
                            += 1usize
                                + ::micropb::size::sizeof_len_record(
                                    val_ref.compute_size(),
                                );
                    }
                }
                {
                    if !self.r#row_major.is_empty() {
                        let len = self.r#row_major.len() * 8usize;
                        size += 1usize + ::micropb::size::sizeof_len_record(len);
                    }
                }
                size
            }
        }
        /// Inner types for `Covariance`
        pub mod Covariance_ {
            /// Compact bitfield for tracking presence of optional and message fields
            #[derive(Debug, Default, PartialEq, Clone, Copy)]
            pub struct _Hazzer([u8; 1]);
            impl _Hazzer {
                /// New hazzer with all fields set to off
                #[inline]
                pub const fn _new() -> Self {
                    Self([0; 1])
                }
                /// Query presence of `time`
                #[inline]
                pub const fn r#time(&self) -> bool {
                    (self.0[0] & 1) != 0
                }
                /// Set presence of `time`
                #[inline]
                pub const fn set_time(&mut self) -> &mut Self {
                    let elem = &mut self.0[0];
                    *elem |= 1;
                    self
                }
                /// Clear presence of `time`
                #[inline]
                pub const fn clear_time(&mut self) -> &mut Self {
                    let elem = &mut self.0[0];
                    *elem &= !1;
                    self
                }
                /// Builder method that sets the presence of `time`. Useful for initializing the Hazzer.
                #[inline]
                pub const fn init_time(mut self) -> Self {
                    self.set_time();
                    self
                }
            }
        }
        /// Continuous-time IMU stochastic error parameters.
        ///
        /// Biases and scale factors are first-order Gauss-Markov with correlation time
        /// `correlation_time_s`; the random walks are white noise on rate and specific
        /// force.
        #[derive(Debug, Default, Clone, Copy)]
        pub struct ImuNoise {
            /// Angle random walk, rad/sqrt(s). Datasheets usually quote deg/sqrt(hour):
            /// multiply by pi/180/60.
            ///
            /// *Note:* The presence of this field is tracked separately in the `_has` field. It's recommended to access this field via the accessor rather than directly.
            pub r#gyro_arw: Vec3,
            /// Velocity random walk, (m/s)/sqrt(s). Datasheets usually quote
            /// m/s/sqrt(hour): divide by 60.
            ///
            /// *Note:* The presence of this field is tracked separately in the `_has` field. It's recommended to access this field via the accessor rather than directly.
            pub r#accel_vrw: Vec3,
            /// Gyro bias process standard deviation, rad/s.
            ///
            /// *Note:* The presence of this field is tracked separately in the `_has` field. It's recommended to access this field via the accessor rather than directly.
            pub r#gyro_bias_std_rps: Vec3,
            /// Accelerometer bias process standard deviation, m/s^2.
            ///
            /// *Note:* The presence of this field is tracked separately in the `_has` field. It's recommended to access this field via the accessor rather than directly.
            pub r#accel_bias_std_mps2: Vec3,
            /// Gyro scale-factor process standard deviation, dimensionless.
            ///
            /// *Note:* The presence of this field is tracked separately in the `_has` field. It's recommended to access this field via the accessor rather than directly.
            pub r#gyro_scale_std: Vec3,
            /// Accelerometer scale-factor process standard deviation, dimensionless.
            ///
            /// *Note:* The presence of this field is tracked separately in the `_has` field. It's recommended to access this field via the accessor rather than directly.
            pub r#accel_scale_std: Vec3,
            /// Gauss-Markov correlation time, seconds. Must be > 0.
            pub r#correlation_time_s: f64,
            /// Tracks presence of optional and message fields
            pub _has: ImuNoise_::_Hazzer,
        }
        impl ::core::cmp::PartialEq for ImuNoise {
            fn eq(&self, other: &Self) -> bool {
                let mut ret = true;
                ret &= (self.r#gyro_arw() == other.r#gyro_arw());
                ret &= (self.r#accel_vrw() == other.r#accel_vrw());
                ret &= (self.r#gyro_bias_std_rps() == other.r#gyro_bias_std_rps());
                ret &= (self.r#accel_bias_std_mps2() == other.r#accel_bias_std_mps2());
                ret &= (self.r#gyro_scale_std() == other.r#gyro_scale_std());
                ret &= (self.r#accel_scale_std() == other.r#accel_scale_std());
                ret &= (self.r#correlation_time_s == other.r#correlation_time_s);
                ret
            }
        }
        impl ImuNoise {
            /// Return a reference to `gyro_arw` as an `Option`
            #[inline]
            pub fn r#gyro_arw(&self) -> ::core::option::Option<&Vec3> {
                self._has.r#gyro_arw().then_some(&self.r#gyro_arw)
            }
            /// Set the value and presence of `gyro_arw`
            #[inline]
            pub fn set_gyro_arw(&mut self, value: Vec3) -> &mut Self {
                self._has.set_gyro_arw();
                self.r#gyro_arw = value.into();
                self
            }
            /// Return a mutable reference to `gyro_arw` as an `Option`
            #[inline]
            pub fn mut_gyro_arw(&mut self) -> ::core::option::Option<&mut Vec3> {
                self._has.r#gyro_arw().then_some(&mut self.r#gyro_arw)
            }
            /// Clear the presence of `gyro_arw`
            #[inline]
            pub fn clear_gyro_arw(&mut self) -> &mut Self {
                self._has.clear_gyro_arw();
                self
            }
            /// Take the value of `gyro_arw` and clear its presence
            #[inline]
            pub fn take_gyro_arw(&mut self) -> ::core::option::Option<Vec3> {
                let val = self
                    ._has
                    .r#gyro_arw()
                    .then(|| ::core::mem::take(&mut self.r#gyro_arw));
                self._has.clear_gyro_arw();
                val
            }
            /// Builder method that sets the value of `gyro_arw`. Useful for initializing the message.
            #[inline]
            pub fn init_gyro_arw(mut self, value: Vec3) -> Self {
                self.set_gyro_arw(value);
                self
            }
            /// Return a reference to `accel_vrw` as an `Option`
            #[inline]
            pub fn r#accel_vrw(&self) -> ::core::option::Option<&Vec3> {
                self._has.r#accel_vrw().then_some(&self.r#accel_vrw)
            }
            /// Set the value and presence of `accel_vrw`
            #[inline]
            pub fn set_accel_vrw(&mut self, value: Vec3) -> &mut Self {
                self._has.set_accel_vrw();
                self.r#accel_vrw = value.into();
                self
            }
            /// Return a mutable reference to `accel_vrw` as an `Option`
            #[inline]
            pub fn mut_accel_vrw(&mut self) -> ::core::option::Option<&mut Vec3> {
                self._has.r#accel_vrw().then_some(&mut self.r#accel_vrw)
            }
            /// Clear the presence of `accel_vrw`
            #[inline]
            pub fn clear_accel_vrw(&mut self) -> &mut Self {
                self._has.clear_accel_vrw();
                self
            }
            /// Take the value of `accel_vrw` and clear its presence
            #[inline]
            pub fn take_accel_vrw(&mut self) -> ::core::option::Option<Vec3> {
                let val = self
                    ._has
                    .r#accel_vrw()
                    .then(|| ::core::mem::take(&mut self.r#accel_vrw));
                self._has.clear_accel_vrw();
                val
            }
            /// Builder method that sets the value of `accel_vrw`. Useful for initializing the message.
            #[inline]
            pub fn init_accel_vrw(mut self, value: Vec3) -> Self {
                self.set_accel_vrw(value);
                self
            }
            /// Return a reference to `gyro_bias_std_rps` as an `Option`
            #[inline]
            pub fn r#gyro_bias_std_rps(&self) -> ::core::option::Option<&Vec3> {
                self._has.r#gyro_bias_std_rps().then_some(&self.r#gyro_bias_std_rps)
            }
            /// Set the value and presence of `gyro_bias_std_rps`
            #[inline]
            pub fn set_gyro_bias_std_rps(&mut self, value: Vec3) -> &mut Self {
                self._has.set_gyro_bias_std_rps();
                self.r#gyro_bias_std_rps = value.into();
                self
            }
            /// Return a mutable reference to `gyro_bias_std_rps` as an `Option`
            #[inline]
            pub fn mut_gyro_bias_std_rps(
                &mut self,
            ) -> ::core::option::Option<&mut Vec3> {
                self._has.r#gyro_bias_std_rps().then_some(&mut self.r#gyro_bias_std_rps)
            }
            /// Clear the presence of `gyro_bias_std_rps`
            #[inline]
            pub fn clear_gyro_bias_std_rps(&mut self) -> &mut Self {
                self._has.clear_gyro_bias_std_rps();
                self
            }
            /// Take the value of `gyro_bias_std_rps` and clear its presence
            #[inline]
            pub fn take_gyro_bias_std_rps(&mut self) -> ::core::option::Option<Vec3> {
                let val = self
                    ._has
                    .r#gyro_bias_std_rps()
                    .then(|| ::core::mem::take(&mut self.r#gyro_bias_std_rps));
                self._has.clear_gyro_bias_std_rps();
                val
            }
            /// Builder method that sets the value of `gyro_bias_std_rps`. Useful for initializing the message.
            #[inline]
            pub fn init_gyro_bias_std_rps(mut self, value: Vec3) -> Self {
                self.set_gyro_bias_std_rps(value);
                self
            }
            /// Return a reference to `accel_bias_std_mps2` as an `Option`
            #[inline]
            pub fn r#accel_bias_std_mps2(&self) -> ::core::option::Option<&Vec3> {
                self._has.r#accel_bias_std_mps2().then_some(&self.r#accel_bias_std_mps2)
            }
            /// Set the value and presence of `accel_bias_std_mps2`
            #[inline]
            pub fn set_accel_bias_std_mps2(&mut self, value: Vec3) -> &mut Self {
                self._has.set_accel_bias_std_mps2();
                self.r#accel_bias_std_mps2 = value.into();
                self
            }
            /// Return a mutable reference to `accel_bias_std_mps2` as an `Option`
            #[inline]
            pub fn mut_accel_bias_std_mps2(
                &mut self,
            ) -> ::core::option::Option<&mut Vec3> {
                self._has
                    .r#accel_bias_std_mps2()
                    .then_some(&mut self.r#accel_bias_std_mps2)
            }
            /// Clear the presence of `accel_bias_std_mps2`
            #[inline]
            pub fn clear_accel_bias_std_mps2(&mut self) -> &mut Self {
                self._has.clear_accel_bias_std_mps2();
                self
            }
            /// Take the value of `accel_bias_std_mps2` and clear its presence
            #[inline]
            pub fn take_accel_bias_std_mps2(&mut self) -> ::core::option::Option<Vec3> {
                let val = self
                    ._has
                    .r#accel_bias_std_mps2()
                    .then(|| ::core::mem::take(&mut self.r#accel_bias_std_mps2));
                self._has.clear_accel_bias_std_mps2();
                val
            }
            /// Builder method that sets the value of `accel_bias_std_mps2`. Useful for initializing the message.
            #[inline]
            pub fn init_accel_bias_std_mps2(mut self, value: Vec3) -> Self {
                self.set_accel_bias_std_mps2(value);
                self
            }
            /// Return a reference to `gyro_scale_std` as an `Option`
            #[inline]
            pub fn r#gyro_scale_std(&self) -> ::core::option::Option<&Vec3> {
                self._has.r#gyro_scale_std().then_some(&self.r#gyro_scale_std)
            }
            /// Set the value and presence of `gyro_scale_std`
            #[inline]
            pub fn set_gyro_scale_std(&mut self, value: Vec3) -> &mut Self {
                self._has.set_gyro_scale_std();
                self.r#gyro_scale_std = value.into();
                self
            }
            /// Return a mutable reference to `gyro_scale_std` as an `Option`
            #[inline]
            pub fn mut_gyro_scale_std(&mut self) -> ::core::option::Option<&mut Vec3> {
                self._has.r#gyro_scale_std().then_some(&mut self.r#gyro_scale_std)
            }
            /// Clear the presence of `gyro_scale_std`
            #[inline]
            pub fn clear_gyro_scale_std(&mut self) -> &mut Self {
                self._has.clear_gyro_scale_std();
                self
            }
            /// Take the value of `gyro_scale_std` and clear its presence
            #[inline]
            pub fn take_gyro_scale_std(&mut self) -> ::core::option::Option<Vec3> {
                let val = self
                    ._has
                    .r#gyro_scale_std()
                    .then(|| ::core::mem::take(&mut self.r#gyro_scale_std));
                self._has.clear_gyro_scale_std();
                val
            }
            /// Builder method that sets the value of `gyro_scale_std`. Useful for initializing the message.
            #[inline]
            pub fn init_gyro_scale_std(mut self, value: Vec3) -> Self {
                self.set_gyro_scale_std(value);
                self
            }
            /// Return a reference to `accel_scale_std` as an `Option`
            #[inline]
            pub fn r#accel_scale_std(&self) -> ::core::option::Option<&Vec3> {
                self._has.r#accel_scale_std().then_some(&self.r#accel_scale_std)
            }
            /// Set the value and presence of `accel_scale_std`
            #[inline]
            pub fn set_accel_scale_std(&mut self, value: Vec3) -> &mut Self {
                self._has.set_accel_scale_std();
                self.r#accel_scale_std = value.into();
                self
            }
            /// Return a mutable reference to `accel_scale_std` as an `Option`
            #[inline]
            pub fn mut_accel_scale_std(&mut self) -> ::core::option::Option<&mut Vec3> {
                self._has.r#accel_scale_std().then_some(&mut self.r#accel_scale_std)
            }
            /// Clear the presence of `accel_scale_std`
            #[inline]
            pub fn clear_accel_scale_std(&mut self) -> &mut Self {
                self._has.clear_accel_scale_std();
                self
            }
            /// Take the value of `accel_scale_std` and clear its presence
            #[inline]
            pub fn take_accel_scale_std(&mut self) -> ::core::option::Option<Vec3> {
                let val = self
                    ._has
                    .r#accel_scale_std()
                    .then(|| ::core::mem::take(&mut self.r#accel_scale_std));
                self._has.clear_accel_scale_std();
                val
            }
            /// Builder method that sets the value of `accel_scale_std`. Useful for initializing the message.
            #[inline]
            pub fn init_accel_scale_std(mut self, value: Vec3) -> Self {
                self.set_accel_scale_std(value);
                self
            }
            /// Return a reference to `correlation_time_s`
            #[inline]
            pub fn r#correlation_time_s(&self) -> &f64 {
                &self.r#correlation_time_s
            }
            /// Return a mutable reference to `correlation_time_s`
            #[inline]
            pub fn mut_correlation_time_s(&mut self) -> &mut f64 {
                &mut self.r#correlation_time_s
            }
            /// Set the value of `correlation_time_s`
            #[inline]
            pub fn set_correlation_time_s(&mut self, value: f64) -> &mut Self {
                self.r#correlation_time_s = value.into();
                self
            }
            /// Builder method that sets the value of `correlation_time_s`. Useful for initializing the message.
            #[inline]
            pub fn init_correlation_time_s(mut self, value: f64) -> Self {
                self.r#correlation_time_s = value.into();
                self
            }
        }
        impl ::micropb::MessageDecode for ImuNoise {
            fn decode<IMPL_MICROPB_READ: ::micropb::PbRead>(
                &mut self,
                decoder: &mut ::micropb::PbDecoder<IMPL_MICROPB_READ>,
                len: usize,
            ) -> Result<(), ::micropb::DecodeError<IMPL_MICROPB_READ::Error>> {
                use ::micropb::{PbBytes, PbString, PbVec, PbMap, FieldDecode};
                let before = decoder.bytes_read();
                while decoder.bytes_read() - before < len {
                    let tag = decoder.decode_tag()?;
                    match tag.field_num() {
                        0 => return Err(::micropb::DecodeError::ZeroField),
                        1u32 => {
                            let mut_ref = &mut self.r#gyro_arw;
                            {
                                mut_ref.decode_len_delimited(decoder)?;
                            };
                            self._has.set_gyro_arw();
                        }
                        2u32 => {
                            let mut_ref = &mut self.r#accel_vrw;
                            {
                                mut_ref.decode_len_delimited(decoder)?;
                            };
                            self._has.set_accel_vrw();
                        }
                        3u32 => {
                            let mut_ref = &mut self.r#gyro_bias_std_rps;
                            {
                                mut_ref.decode_len_delimited(decoder)?;
                            };
                            self._has.set_gyro_bias_std_rps();
                        }
                        4u32 => {
                            let mut_ref = &mut self.r#accel_bias_std_mps2;
                            {
                                mut_ref.decode_len_delimited(decoder)?;
                            };
                            self._has.set_accel_bias_std_mps2();
                        }
                        5u32 => {
                            let mut_ref = &mut self.r#gyro_scale_std;
                            {
                                mut_ref.decode_len_delimited(decoder)?;
                            };
                            self._has.set_gyro_scale_std();
                        }
                        6u32 => {
                            let mut_ref = &mut self.r#accel_scale_std;
                            {
                                mut_ref.decode_len_delimited(decoder)?;
                            };
                            self._has.set_accel_scale_std();
                        }
                        7u32 => {
                            let mut_ref = &mut self.r#correlation_time_s;
                            {
                                let val = decoder.decode_double()?;
                                let val_ref = &val;
                                if *val_ref != 0.0 {
                                    *mut_ref = val as _;
                                }
                            };
                        }
                        _ => {
                            decoder.skip_wire_value(tag.wire_type())?;
                        }
                    }
                }
                Ok(())
            }
        }
        impl ::micropb::MessageEncode for ImuNoise {
            const MAX_SIZE: ::core::result::Result<usize, &'static str> = 'msg: {
                let mut max_size = 0;
                match ::micropb::const_map!(
                    ::micropb::const_map!(< Vec3 as ::micropb::MessageEncode >
                    ::MAX_SIZE, | size | ::micropb::size::sizeof_len_record(size)), |
                    size | size + 1usize
                ) {
                    ::core::result::Result::Ok(size) => {
                        max_size += size;
                    }
                    ::core::result::Result::Err(err) => {
                        break 'msg (::core::result::Result::<usize, _>::Err(err));
                    }
                }
                match ::micropb::const_map!(
                    ::micropb::const_map!(< Vec3 as ::micropb::MessageEncode >
                    ::MAX_SIZE, | size | ::micropb::size::sizeof_len_record(size)), |
                    size | size + 1usize
                ) {
                    ::core::result::Result::Ok(size) => {
                        max_size += size;
                    }
                    ::core::result::Result::Err(err) => {
                        break 'msg (::core::result::Result::<usize, _>::Err(err));
                    }
                }
                match ::micropb::const_map!(
                    ::micropb::const_map!(< Vec3 as ::micropb::MessageEncode >
                    ::MAX_SIZE, | size | ::micropb::size::sizeof_len_record(size)), |
                    size | size + 1usize
                ) {
                    ::core::result::Result::Ok(size) => {
                        max_size += size;
                    }
                    ::core::result::Result::Err(err) => {
                        break 'msg (::core::result::Result::<usize, _>::Err(err));
                    }
                }
                match ::micropb::const_map!(
                    ::micropb::const_map!(< Vec3 as ::micropb::MessageEncode >
                    ::MAX_SIZE, | size | ::micropb::size::sizeof_len_record(size)), |
                    size | size + 1usize
                ) {
                    ::core::result::Result::Ok(size) => {
                        max_size += size;
                    }
                    ::core::result::Result::Err(err) => {
                        break 'msg (::core::result::Result::<usize, _>::Err(err));
                    }
                }
                match ::micropb::const_map!(
                    ::micropb::const_map!(< Vec3 as ::micropb::MessageEncode >
                    ::MAX_SIZE, | size | ::micropb::size::sizeof_len_record(size)), |
                    size | size + 1usize
                ) {
                    ::core::result::Result::Ok(size) => {
                        max_size += size;
                    }
                    ::core::result::Result::Err(err) => {
                        break 'msg (::core::result::Result::<usize, _>::Err(err));
                    }
                }
                match ::micropb::const_map!(
                    ::micropb::const_map!(< Vec3 as ::micropb::MessageEncode >
                    ::MAX_SIZE, | size | ::micropb::size::sizeof_len_record(size)), |
                    size | size + 1usize
                ) {
                    ::core::result::Result::Ok(size) => {
                        max_size += size;
                    }
                    ::core::result::Result::Err(err) => {
                        break 'msg (::core::result::Result::<usize, _>::Err(err));
                    }
                }
                match ::micropb::const_map!(
                    ::core::result::Result::Ok(8usize), | size | size + 1usize
                ) {
                    ::core::result::Result::Ok(size) => {
                        max_size += size;
                    }
                    ::core::result::Result::Err(err) => {
                        break 'msg (::core::result::Result::<usize, _>::Err(err));
                    }
                }
                ::core::result::Result::Ok(max_size)
            };
            fn encode<IMPL_MICROPB_WRITE: ::micropb::PbWrite>(
                &self,
                encoder: &mut ::micropb::PbEncoder<IMPL_MICROPB_WRITE>,
            ) -> Result<(), IMPL_MICROPB_WRITE::Error> {
                use ::micropb::{PbMap, FieldEncode};
                {
                    if let ::core::option::Option::Some(val_ref) = self.r#gyro_arw() {
                        encoder.encode_varint32(10u32)?;
                        val_ref.encode_len_delimited(encoder)?;
                    }
                }
                {
                    if let ::core::option::Option::Some(val_ref) = self.r#accel_vrw() {
                        encoder.encode_varint32(18u32)?;
                        val_ref.encode_len_delimited(encoder)?;
                    }
                }
                {
                    if let ::core::option::Option::Some(val_ref) = self
                        .r#gyro_bias_std_rps()
                    {
                        encoder.encode_varint32(26u32)?;
                        val_ref.encode_len_delimited(encoder)?;
                    }
                }
                {
                    if let ::core::option::Option::Some(val_ref) = self
                        .r#accel_bias_std_mps2()
                    {
                        encoder.encode_varint32(34u32)?;
                        val_ref.encode_len_delimited(encoder)?;
                    }
                }
                {
                    if let ::core::option::Option::Some(val_ref) = self
                        .r#gyro_scale_std()
                    {
                        encoder.encode_varint32(42u32)?;
                        val_ref.encode_len_delimited(encoder)?;
                    }
                }
                {
                    if let ::core::option::Option::Some(val_ref) = self
                        .r#accel_scale_std()
                    {
                        encoder.encode_varint32(50u32)?;
                        val_ref.encode_len_delimited(encoder)?;
                    }
                }
                {
                    let val_ref = &self.r#correlation_time_s;
                    if *val_ref != 0.0 {
                        encoder.encode_varint32(57u32)?;
                        encoder.encode_double(*val_ref)?;
                    }
                }
                Ok(())
            }
            fn compute_size(&self) -> usize {
                use ::micropb::{PbMap, FieldEncode};
                let mut size = 0;
                {
                    if let ::core::option::Option::Some(val_ref) = self.r#gyro_arw() {
                        size
                            += 1usize
                                + ::micropb::size::sizeof_len_record(
                                    val_ref.compute_size(),
                                );
                    }
                }
                {
                    if let ::core::option::Option::Some(val_ref) = self.r#accel_vrw() {
                        size
                            += 1usize
                                + ::micropb::size::sizeof_len_record(
                                    val_ref.compute_size(),
                                );
                    }
                }
                {
                    if let ::core::option::Option::Some(val_ref) = self
                        .r#gyro_bias_std_rps()
                    {
                        size
                            += 1usize
                                + ::micropb::size::sizeof_len_record(
                                    val_ref.compute_size(),
                                );
                    }
                }
                {
                    if let ::core::option::Option::Some(val_ref) = self
                        .r#accel_bias_std_mps2()
                    {
                        size
                            += 1usize
                                + ::micropb::size::sizeof_len_record(
                                    val_ref.compute_size(),
                                );
                    }
                }
                {
                    if let ::core::option::Option::Some(val_ref) = self
                        .r#gyro_scale_std()
                    {
                        size
                            += 1usize
                                + ::micropb::size::sizeof_len_record(
                                    val_ref.compute_size(),
                                );
                    }
                }
                {
                    if let ::core::option::Option::Some(val_ref) = self
                        .r#accel_scale_std()
                    {
                        size
                            += 1usize
                                + ::micropb::size::sizeof_len_record(
                                    val_ref.compute_size(),
                                );
                    }
                }
                {
                    let val_ref = &self.r#correlation_time_s;
                    if *val_ref != 0.0 {
                        size += 1usize + 8;
                    }
                }
                size
            }
        }
        /// Inner types for `ImuNoise`
        pub mod ImuNoise_ {
            /// Compact bitfield for tracking presence of optional and message fields
            #[derive(Debug, Default, PartialEq, Clone, Copy)]
            pub struct _Hazzer([u8; 1]);
            impl _Hazzer {
                /// New hazzer with all fields set to off
                #[inline]
                pub const fn _new() -> Self {
                    Self([0; 1])
                }
                /// Query presence of `gyro_arw`
                #[inline]
                pub const fn r#gyro_arw(&self) -> bool {
                    (self.0[0] & 1) != 0
                }
                /// Set presence of `gyro_arw`
                #[inline]
                pub const fn set_gyro_arw(&mut self) -> &mut Self {
                    let elem = &mut self.0[0];
                    *elem |= 1;
                    self
                }
                /// Clear presence of `gyro_arw`
                #[inline]
                pub const fn clear_gyro_arw(&mut self) -> &mut Self {
                    let elem = &mut self.0[0];
                    *elem &= !1;
                    self
                }
                /// Builder method that sets the presence of `gyro_arw`. Useful for initializing the Hazzer.
                #[inline]
                pub const fn init_gyro_arw(mut self) -> Self {
                    self.set_gyro_arw();
                    self
                }
                /// Query presence of `accel_vrw`
                #[inline]
                pub const fn r#accel_vrw(&self) -> bool {
                    (self.0[0] & 2) != 0
                }
                /// Set presence of `accel_vrw`
                #[inline]
                pub const fn set_accel_vrw(&mut self) -> &mut Self {
                    let elem = &mut self.0[0];
                    *elem |= 2;
                    self
                }
                /// Clear presence of `accel_vrw`
                #[inline]
                pub const fn clear_accel_vrw(&mut self) -> &mut Self {
                    let elem = &mut self.0[0];
                    *elem &= !2;
                    self
                }
                /// Builder method that sets the presence of `accel_vrw`. Useful for initializing the Hazzer.
                #[inline]
                pub const fn init_accel_vrw(mut self) -> Self {
                    self.set_accel_vrw();
                    self
                }
                /// Query presence of `gyro_bias_std_rps`
                #[inline]
                pub const fn r#gyro_bias_std_rps(&self) -> bool {
                    (self.0[0] & 4) != 0
                }
                /// Set presence of `gyro_bias_std_rps`
                #[inline]
                pub const fn set_gyro_bias_std_rps(&mut self) -> &mut Self {
                    let elem = &mut self.0[0];
                    *elem |= 4;
                    self
                }
                /// Clear presence of `gyro_bias_std_rps`
                #[inline]
                pub const fn clear_gyro_bias_std_rps(&mut self) -> &mut Self {
                    let elem = &mut self.0[0];
                    *elem &= !4;
                    self
                }
                /// Builder method that sets the presence of `gyro_bias_std_rps`. Useful for initializing the Hazzer.
                #[inline]
                pub const fn init_gyro_bias_std_rps(mut self) -> Self {
                    self.set_gyro_bias_std_rps();
                    self
                }
                /// Query presence of `accel_bias_std_mps2`
                #[inline]
                pub const fn r#accel_bias_std_mps2(&self) -> bool {
                    (self.0[0] & 8) != 0
                }
                /// Set presence of `accel_bias_std_mps2`
                #[inline]
                pub const fn set_accel_bias_std_mps2(&mut self) -> &mut Self {
                    let elem = &mut self.0[0];
                    *elem |= 8;
                    self
                }
                /// Clear presence of `accel_bias_std_mps2`
                #[inline]
                pub const fn clear_accel_bias_std_mps2(&mut self) -> &mut Self {
                    let elem = &mut self.0[0];
                    *elem &= !8;
                    self
                }
                /// Builder method that sets the presence of `accel_bias_std_mps2`. Useful for initializing the Hazzer.
                #[inline]
                pub const fn init_accel_bias_std_mps2(mut self) -> Self {
                    self.set_accel_bias_std_mps2();
                    self
                }
                /// Query presence of `gyro_scale_std`
                #[inline]
                pub const fn r#gyro_scale_std(&self) -> bool {
                    (self.0[0] & 16) != 0
                }
                /// Set presence of `gyro_scale_std`
                #[inline]
                pub const fn set_gyro_scale_std(&mut self) -> &mut Self {
                    let elem = &mut self.0[0];
                    *elem |= 16;
                    self
                }
                /// Clear presence of `gyro_scale_std`
                #[inline]
                pub const fn clear_gyro_scale_std(&mut self) -> &mut Self {
                    let elem = &mut self.0[0];
                    *elem &= !16;
                    self
                }
                /// Builder method that sets the presence of `gyro_scale_std`. Useful for initializing the Hazzer.
                #[inline]
                pub const fn init_gyro_scale_std(mut self) -> Self {
                    self.set_gyro_scale_std();
                    self
                }
                /// Query presence of `accel_scale_std`
                #[inline]
                pub const fn r#accel_scale_std(&self) -> bool {
                    (self.0[0] & 32) != 0
                }
                /// Set presence of `accel_scale_std`
                #[inline]
                pub const fn set_accel_scale_std(&mut self) -> &mut Self {
                    let elem = &mut self.0[0];
                    *elem |= 32;
                    self
                }
                /// Clear presence of `accel_scale_std`
                #[inline]
                pub const fn clear_accel_scale_std(&mut self) -> &mut Self {
                    let elem = &mut self.0[0];
                    *elem &= !32;
                    self
                }
                /// Builder method that sets the presence of `accel_scale_std`. Useful for initializing the Hazzer.
                #[inline]
                pub const fn init_accel_scale_std(mut self) -> Self {
                    self.set_accel_scale_std();
                    self
                }
            }
        }
        /// Everything the engine needs to start.
        ///
        /// The initial standard deviations matter as much as the initial state: they set
        /// the diagonal of P and therefore how hard the first fixes are allowed to pull
        /// the solution. All must be strictly positive.
        #[derive(Debug, Default, Clone, Copy)]
        pub struct GinsOptions {
            /// *Note:* The presence of this field is tracked separately in the `_has` field. It's recommended to access this field via the accessor rather than directly.
            pub r#initial_position: Lla,
            /// *Note:* The presence of this field is tracked separately in the `_has` field. It's recommended to access this field via the accessor rather than directly.
            pub r#initial_velocity_mps: Ned,
            /// *Note:* The presence of this field is tracked separately in the `_has` field. It's recommended to access this field via the accessor rather than directly.
            pub r#initial_attitude: Euler,
            /// *Note:* The presence of this field is tracked separately in the `_has` field. It's recommended to access this field via the accessor rather than directly.
            pub r#initial_imu_error: ImuError,
            /// *Note:* The presence of this field is tracked separately in the `_has` field. It's recommended to access this field via the accessor rather than directly.
            pub r#initial_position_std_m: Vec3,
            /// *Note:* The presence of this field is tracked separately in the `_has` field. It's recommended to access this field via the accessor rather than directly.
            pub r#initial_velocity_std_mps: Vec3,
            /// One-sigma initial attitude uncertainty, radians (roll, pitch, yaw).
            ///
            /// *Note:* The presence of this field is tracked separately in the `_has` field. It's recommended to access this field via the accessor rather than directly.
            pub r#initial_attitude_std_rad: Vec3,
            /// *Note:* The presence of this field is tracked separately in the `_has` field. It's recommended to access this field via the accessor rather than directly.
            pub r#initial_gyro_bias_std_rps: Vec3,
            /// *Note:* The presence of this field is tracked separately in the `_has` field. It's recommended to access this field via the accessor rather than directly.
            pub r#initial_accel_bias_std_mps2: Vec3,
            /// *Note:* The presence of this field is tracked separately in the `_has` field. It's recommended to access this field via the accessor rather than directly.
            pub r#initial_gyro_scale_std: Vec3,
            /// *Note:* The presence of this field is tracked separately in the `_has` field. It's recommended to access this field via the accessor rather than directly.
            pub r#initial_accel_scale_std: Vec3,
            /// *Note:* The presence of this field is tracked separately in the `_has` field. It's recommended to access this field via the accessor rather than directly.
            pub r#imu_noise: ImuNoise,
            /// GNSS antenna phase centre in the BODY frame, metres (forward, right, down
            /// from the IMU reference point). A sign error here appears as a
            /// heading-dependent position bias.
            ///
            /// *Note:* The presence of this field is tracked separately in the `_has` field. It's recommended to access this field via the accessor rather than directly.
            pub r#antenna_lever_arm_m: Vec3,
            /// Consecutive chi-squared gate rejections tolerated before the covariance is
            /// inflated. A filter that rejects every measurement has a covariance that is
            /// confident and wrong, and discards exactly the information that would fix
            /// it. Zero disables the recovery.
            pub r#max_consecutive_rejections: u32,
            /// Covariance scale factor applied when that limit is reached. Must be >= 1.
            pub r#rejection_inflation: f64,
            /// Tracks presence of optional and message fields
            pub _has: GinsOptions_::_Hazzer,
        }
        impl ::core::cmp::PartialEq for GinsOptions {
            fn eq(&self, other: &Self) -> bool {
                let mut ret = true;
                ret &= (self.r#initial_position() == other.r#initial_position());
                ret &= (self.r#initial_velocity_mps() == other.r#initial_velocity_mps());
                ret &= (self.r#initial_attitude() == other.r#initial_attitude());
                ret &= (self.r#initial_imu_error() == other.r#initial_imu_error());
                ret
                    &= (self.r#initial_position_std_m()
                        == other.r#initial_position_std_m());
                ret
                    &= (self.r#initial_velocity_std_mps()
                        == other.r#initial_velocity_std_mps());
                ret
                    &= (self.r#initial_attitude_std_rad()
                        == other.r#initial_attitude_std_rad());
                ret
                    &= (self.r#initial_gyro_bias_std_rps()
                        == other.r#initial_gyro_bias_std_rps());
                ret
                    &= (self.r#initial_accel_bias_std_mps2()
                        == other.r#initial_accel_bias_std_mps2());
                ret
                    &= (self.r#initial_gyro_scale_std()
                        == other.r#initial_gyro_scale_std());
                ret
                    &= (self.r#initial_accel_scale_std()
                        == other.r#initial_accel_scale_std());
                ret &= (self.r#imu_noise() == other.r#imu_noise());
                ret &= (self.r#antenna_lever_arm_m() == other.r#antenna_lever_arm_m());
                ret
                    &= (self.r#max_consecutive_rejections
                        == other.r#max_consecutive_rejections);
                ret &= (self.r#rejection_inflation == other.r#rejection_inflation);
                ret
            }
        }
        impl GinsOptions {
            /// Return a reference to `initial_position` as an `Option`
            #[inline]
            pub fn r#initial_position(&self) -> ::core::option::Option<&Lla> {
                self._has.r#initial_position().then_some(&self.r#initial_position)
            }
            /// Set the value and presence of `initial_position`
            #[inline]
            pub fn set_initial_position(&mut self, value: Lla) -> &mut Self {
                self._has.set_initial_position();
                self.r#initial_position = value.into();
                self
            }
            /// Return a mutable reference to `initial_position` as an `Option`
            #[inline]
            pub fn mut_initial_position(&mut self) -> ::core::option::Option<&mut Lla> {
                self._has.r#initial_position().then_some(&mut self.r#initial_position)
            }
            /// Clear the presence of `initial_position`
            #[inline]
            pub fn clear_initial_position(&mut self) -> &mut Self {
                self._has.clear_initial_position();
                self
            }
            /// Take the value of `initial_position` and clear its presence
            #[inline]
            pub fn take_initial_position(&mut self) -> ::core::option::Option<Lla> {
                let val = self
                    ._has
                    .r#initial_position()
                    .then(|| ::core::mem::take(&mut self.r#initial_position));
                self._has.clear_initial_position();
                val
            }
            /// Builder method that sets the value of `initial_position`. Useful for initializing the message.
            #[inline]
            pub fn init_initial_position(mut self, value: Lla) -> Self {
                self.set_initial_position(value);
                self
            }
            /// Return a reference to `initial_velocity_mps` as an `Option`
            #[inline]
            pub fn r#initial_velocity_mps(&self) -> ::core::option::Option<&Ned> {
                self._has
                    .r#initial_velocity_mps()
                    .then_some(&self.r#initial_velocity_mps)
            }
            /// Set the value and presence of `initial_velocity_mps`
            #[inline]
            pub fn set_initial_velocity_mps(&mut self, value: Ned) -> &mut Self {
                self._has.set_initial_velocity_mps();
                self.r#initial_velocity_mps = value.into();
                self
            }
            /// Return a mutable reference to `initial_velocity_mps` as an `Option`
            #[inline]
            pub fn mut_initial_velocity_mps(
                &mut self,
            ) -> ::core::option::Option<&mut Ned> {
                self._has
                    .r#initial_velocity_mps()
                    .then_some(&mut self.r#initial_velocity_mps)
            }
            /// Clear the presence of `initial_velocity_mps`
            #[inline]
            pub fn clear_initial_velocity_mps(&mut self) -> &mut Self {
                self._has.clear_initial_velocity_mps();
                self
            }
            /// Take the value of `initial_velocity_mps` and clear its presence
            #[inline]
            pub fn take_initial_velocity_mps(&mut self) -> ::core::option::Option<Ned> {
                let val = self
                    ._has
                    .r#initial_velocity_mps()
                    .then(|| ::core::mem::take(&mut self.r#initial_velocity_mps));
                self._has.clear_initial_velocity_mps();
                val
            }
            /// Builder method that sets the value of `initial_velocity_mps`. Useful for initializing the message.
            #[inline]
            pub fn init_initial_velocity_mps(mut self, value: Ned) -> Self {
                self.set_initial_velocity_mps(value);
                self
            }
            /// Return a reference to `initial_attitude` as an `Option`
            #[inline]
            pub fn r#initial_attitude(&self) -> ::core::option::Option<&Euler> {
                self._has.r#initial_attitude().then_some(&self.r#initial_attitude)
            }
            /// Set the value and presence of `initial_attitude`
            #[inline]
            pub fn set_initial_attitude(&mut self, value: Euler) -> &mut Self {
                self._has.set_initial_attitude();
                self.r#initial_attitude = value.into();
                self
            }
            /// Return a mutable reference to `initial_attitude` as an `Option`
            #[inline]
            pub fn mut_initial_attitude(
                &mut self,
            ) -> ::core::option::Option<&mut Euler> {
                self._has.r#initial_attitude().then_some(&mut self.r#initial_attitude)
            }
            /// Clear the presence of `initial_attitude`
            #[inline]
            pub fn clear_initial_attitude(&mut self) -> &mut Self {
                self._has.clear_initial_attitude();
                self
            }
            /// Take the value of `initial_attitude` and clear its presence
            #[inline]
            pub fn take_initial_attitude(&mut self) -> ::core::option::Option<Euler> {
                let val = self
                    ._has
                    .r#initial_attitude()
                    .then(|| ::core::mem::take(&mut self.r#initial_attitude));
                self._has.clear_initial_attitude();
                val
            }
            /// Builder method that sets the value of `initial_attitude`. Useful for initializing the message.
            #[inline]
            pub fn init_initial_attitude(mut self, value: Euler) -> Self {
                self.set_initial_attitude(value);
                self
            }
            /// Return a reference to `initial_imu_error` as an `Option`
            #[inline]
            pub fn r#initial_imu_error(&self) -> ::core::option::Option<&ImuError> {
                self._has.r#initial_imu_error().then_some(&self.r#initial_imu_error)
            }
            /// Set the value and presence of `initial_imu_error`
            #[inline]
            pub fn set_initial_imu_error(&mut self, value: ImuError) -> &mut Self {
                self._has.set_initial_imu_error();
                self.r#initial_imu_error = value.into();
                self
            }
            /// Return a mutable reference to `initial_imu_error` as an `Option`
            #[inline]
            pub fn mut_initial_imu_error(
                &mut self,
            ) -> ::core::option::Option<&mut ImuError> {
                self._has.r#initial_imu_error().then_some(&mut self.r#initial_imu_error)
            }
            /// Clear the presence of `initial_imu_error`
            #[inline]
            pub fn clear_initial_imu_error(&mut self) -> &mut Self {
                self._has.clear_initial_imu_error();
                self
            }
            /// Take the value of `initial_imu_error` and clear its presence
            #[inline]
            pub fn take_initial_imu_error(
                &mut self,
            ) -> ::core::option::Option<ImuError> {
                let val = self
                    ._has
                    .r#initial_imu_error()
                    .then(|| ::core::mem::take(&mut self.r#initial_imu_error));
                self._has.clear_initial_imu_error();
                val
            }
            /// Builder method that sets the value of `initial_imu_error`. Useful for initializing the message.
            #[inline]
            pub fn init_initial_imu_error(mut self, value: ImuError) -> Self {
                self.set_initial_imu_error(value);
                self
            }
            /// Return a reference to `initial_position_std_m` as an `Option`
            #[inline]
            pub fn r#initial_position_std_m(&self) -> ::core::option::Option<&Vec3> {
                self._has
                    .r#initial_position_std_m()
                    .then_some(&self.r#initial_position_std_m)
            }
            /// Set the value and presence of `initial_position_std_m`
            #[inline]
            pub fn set_initial_position_std_m(&mut self, value: Vec3) -> &mut Self {
                self._has.set_initial_position_std_m();
                self.r#initial_position_std_m = value.into();
                self
            }
            /// Return a mutable reference to `initial_position_std_m` as an `Option`
            #[inline]
            pub fn mut_initial_position_std_m(
                &mut self,
            ) -> ::core::option::Option<&mut Vec3> {
                self._has
                    .r#initial_position_std_m()
                    .then_some(&mut self.r#initial_position_std_m)
            }
            /// Clear the presence of `initial_position_std_m`
            #[inline]
            pub fn clear_initial_position_std_m(&mut self) -> &mut Self {
                self._has.clear_initial_position_std_m();
                self
            }
            /// Take the value of `initial_position_std_m` and clear its presence
            #[inline]
            pub fn take_initial_position_std_m(
                &mut self,
            ) -> ::core::option::Option<Vec3> {
                let val = self
                    ._has
                    .r#initial_position_std_m()
                    .then(|| ::core::mem::take(&mut self.r#initial_position_std_m));
                self._has.clear_initial_position_std_m();
                val
            }
            /// Builder method that sets the value of `initial_position_std_m`. Useful for initializing the message.
            #[inline]
            pub fn init_initial_position_std_m(mut self, value: Vec3) -> Self {
                self.set_initial_position_std_m(value);
                self
            }
            /// Return a reference to `initial_velocity_std_mps` as an `Option`
            #[inline]
            pub fn r#initial_velocity_std_mps(&self) -> ::core::option::Option<&Vec3> {
                self._has
                    .r#initial_velocity_std_mps()
                    .then_some(&self.r#initial_velocity_std_mps)
            }
            /// Set the value and presence of `initial_velocity_std_mps`
            #[inline]
            pub fn set_initial_velocity_std_mps(&mut self, value: Vec3) -> &mut Self {
                self._has.set_initial_velocity_std_mps();
                self.r#initial_velocity_std_mps = value.into();
                self
            }
            /// Return a mutable reference to `initial_velocity_std_mps` as an `Option`
            #[inline]
            pub fn mut_initial_velocity_std_mps(
                &mut self,
            ) -> ::core::option::Option<&mut Vec3> {
                self._has
                    .r#initial_velocity_std_mps()
                    .then_some(&mut self.r#initial_velocity_std_mps)
            }
            /// Clear the presence of `initial_velocity_std_mps`
            #[inline]
            pub fn clear_initial_velocity_std_mps(&mut self) -> &mut Self {
                self._has.clear_initial_velocity_std_mps();
                self
            }
            /// Take the value of `initial_velocity_std_mps` and clear its presence
            #[inline]
            pub fn take_initial_velocity_std_mps(
                &mut self,
            ) -> ::core::option::Option<Vec3> {
                let val = self
                    ._has
                    .r#initial_velocity_std_mps()
                    .then(|| ::core::mem::take(&mut self.r#initial_velocity_std_mps));
                self._has.clear_initial_velocity_std_mps();
                val
            }
            /// Builder method that sets the value of `initial_velocity_std_mps`. Useful for initializing the message.
            #[inline]
            pub fn init_initial_velocity_std_mps(mut self, value: Vec3) -> Self {
                self.set_initial_velocity_std_mps(value);
                self
            }
            /// Return a reference to `initial_attitude_std_rad` as an `Option`
            #[inline]
            pub fn r#initial_attitude_std_rad(&self) -> ::core::option::Option<&Vec3> {
                self._has
                    .r#initial_attitude_std_rad()
                    .then_some(&self.r#initial_attitude_std_rad)
            }
            /// Set the value and presence of `initial_attitude_std_rad`
            #[inline]
            pub fn set_initial_attitude_std_rad(&mut self, value: Vec3) -> &mut Self {
                self._has.set_initial_attitude_std_rad();
                self.r#initial_attitude_std_rad = value.into();
                self
            }
            /// Return a mutable reference to `initial_attitude_std_rad` as an `Option`
            #[inline]
            pub fn mut_initial_attitude_std_rad(
                &mut self,
            ) -> ::core::option::Option<&mut Vec3> {
                self._has
                    .r#initial_attitude_std_rad()
                    .then_some(&mut self.r#initial_attitude_std_rad)
            }
            /// Clear the presence of `initial_attitude_std_rad`
            #[inline]
            pub fn clear_initial_attitude_std_rad(&mut self) -> &mut Self {
                self._has.clear_initial_attitude_std_rad();
                self
            }
            /// Take the value of `initial_attitude_std_rad` and clear its presence
            #[inline]
            pub fn take_initial_attitude_std_rad(
                &mut self,
            ) -> ::core::option::Option<Vec3> {
                let val = self
                    ._has
                    .r#initial_attitude_std_rad()
                    .then(|| ::core::mem::take(&mut self.r#initial_attitude_std_rad));
                self._has.clear_initial_attitude_std_rad();
                val
            }
            /// Builder method that sets the value of `initial_attitude_std_rad`. Useful for initializing the message.
            #[inline]
            pub fn init_initial_attitude_std_rad(mut self, value: Vec3) -> Self {
                self.set_initial_attitude_std_rad(value);
                self
            }
            /// Return a reference to `initial_gyro_bias_std_rps` as an `Option`
            #[inline]
            pub fn r#initial_gyro_bias_std_rps(&self) -> ::core::option::Option<&Vec3> {
                self._has
                    .r#initial_gyro_bias_std_rps()
                    .then_some(&self.r#initial_gyro_bias_std_rps)
            }
            /// Set the value and presence of `initial_gyro_bias_std_rps`
            #[inline]
            pub fn set_initial_gyro_bias_std_rps(&mut self, value: Vec3) -> &mut Self {
                self._has.set_initial_gyro_bias_std_rps();
                self.r#initial_gyro_bias_std_rps = value.into();
                self
            }
            /// Return a mutable reference to `initial_gyro_bias_std_rps` as an `Option`
            #[inline]
            pub fn mut_initial_gyro_bias_std_rps(
                &mut self,
            ) -> ::core::option::Option<&mut Vec3> {
                self._has
                    .r#initial_gyro_bias_std_rps()
                    .then_some(&mut self.r#initial_gyro_bias_std_rps)
            }
            /// Clear the presence of `initial_gyro_bias_std_rps`
            #[inline]
            pub fn clear_initial_gyro_bias_std_rps(&mut self) -> &mut Self {
                self._has.clear_initial_gyro_bias_std_rps();
                self
            }
            /// Take the value of `initial_gyro_bias_std_rps` and clear its presence
            #[inline]
            pub fn take_initial_gyro_bias_std_rps(
                &mut self,
            ) -> ::core::option::Option<Vec3> {
                let val = self
                    ._has
                    .r#initial_gyro_bias_std_rps()
                    .then(|| ::core::mem::take(&mut self.r#initial_gyro_bias_std_rps));
                self._has.clear_initial_gyro_bias_std_rps();
                val
            }
            /// Builder method that sets the value of `initial_gyro_bias_std_rps`. Useful for initializing the message.
            #[inline]
            pub fn init_initial_gyro_bias_std_rps(mut self, value: Vec3) -> Self {
                self.set_initial_gyro_bias_std_rps(value);
                self
            }
            /// Return a reference to `initial_accel_bias_std_mps2` as an `Option`
            #[inline]
            pub fn r#initial_accel_bias_std_mps2(
                &self,
            ) -> ::core::option::Option<&Vec3> {
                self._has
                    .r#initial_accel_bias_std_mps2()
                    .then_some(&self.r#initial_accel_bias_std_mps2)
            }
            /// Set the value and presence of `initial_accel_bias_std_mps2`
            #[inline]
            pub fn set_initial_accel_bias_std_mps2(&mut self, value: Vec3) -> &mut Self {
                self._has.set_initial_accel_bias_std_mps2();
                self.r#initial_accel_bias_std_mps2 = value.into();
                self
            }
            /// Return a mutable reference to `initial_accel_bias_std_mps2` as an `Option`
            #[inline]
            pub fn mut_initial_accel_bias_std_mps2(
                &mut self,
            ) -> ::core::option::Option<&mut Vec3> {
                self._has
                    .r#initial_accel_bias_std_mps2()
                    .then_some(&mut self.r#initial_accel_bias_std_mps2)
            }
            /// Clear the presence of `initial_accel_bias_std_mps2`
            #[inline]
            pub fn clear_initial_accel_bias_std_mps2(&mut self) -> &mut Self {
                self._has.clear_initial_accel_bias_std_mps2();
                self
            }
            /// Take the value of `initial_accel_bias_std_mps2` and clear its presence
            #[inline]
            pub fn take_initial_accel_bias_std_mps2(
                &mut self,
            ) -> ::core::option::Option<Vec3> {
                let val = self
                    ._has
                    .r#initial_accel_bias_std_mps2()
                    .then(|| ::core::mem::take(&mut self.r#initial_accel_bias_std_mps2));
                self._has.clear_initial_accel_bias_std_mps2();
                val
            }
            /// Builder method that sets the value of `initial_accel_bias_std_mps2`. Useful for initializing the message.
            #[inline]
            pub fn init_initial_accel_bias_std_mps2(mut self, value: Vec3) -> Self {
                self.set_initial_accel_bias_std_mps2(value);
                self
            }
            /// Return a reference to `initial_gyro_scale_std` as an `Option`
            #[inline]
            pub fn r#initial_gyro_scale_std(&self) -> ::core::option::Option<&Vec3> {
                self._has
                    .r#initial_gyro_scale_std()
                    .then_some(&self.r#initial_gyro_scale_std)
            }
            /// Set the value and presence of `initial_gyro_scale_std`
            #[inline]
            pub fn set_initial_gyro_scale_std(&mut self, value: Vec3) -> &mut Self {
                self._has.set_initial_gyro_scale_std();
                self.r#initial_gyro_scale_std = value.into();
                self
            }
            /// Return a mutable reference to `initial_gyro_scale_std` as an `Option`
            #[inline]
            pub fn mut_initial_gyro_scale_std(
                &mut self,
            ) -> ::core::option::Option<&mut Vec3> {
                self._has
                    .r#initial_gyro_scale_std()
                    .then_some(&mut self.r#initial_gyro_scale_std)
            }
            /// Clear the presence of `initial_gyro_scale_std`
            #[inline]
            pub fn clear_initial_gyro_scale_std(&mut self) -> &mut Self {
                self._has.clear_initial_gyro_scale_std();
                self
            }
            /// Take the value of `initial_gyro_scale_std` and clear its presence
            #[inline]
            pub fn take_initial_gyro_scale_std(
                &mut self,
            ) -> ::core::option::Option<Vec3> {
                let val = self
                    ._has
                    .r#initial_gyro_scale_std()
                    .then(|| ::core::mem::take(&mut self.r#initial_gyro_scale_std));
                self._has.clear_initial_gyro_scale_std();
                val
            }
            /// Builder method that sets the value of `initial_gyro_scale_std`. Useful for initializing the message.
            #[inline]
            pub fn init_initial_gyro_scale_std(mut self, value: Vec3) -> Self {
                self.set_initial_gyro_scale_std(value);
                self
            }
            /// Return a reference to `initial_accel_scale_std` as an `Option`
            #[inline]
            pub fn r#initial_accel_scale_std(&self) -> ::core::option::Option<&Vec3> {
                self._has
                    .r#initial_accel_scale_std()
                    .then_some(&self.r#initial_accel_scale_std)
            }
            /// Set the value and presence of `initial_accel_scale_std`
            #[inline]
            pub fn set_initial_accel_scale_std(&mut self, value: Vec3) -> &mut Self {
                self._has.set_initial_accel_scale_std();
                self.r#initial_accel_scale_std = value.into();
                self
            }
            /// Return a mutable reference to `initial_accel_scale_std` as an `Option`
            #[inline]
            pub fn mut_initial_accel_scale_std(
                &mut self,
            ) -> ::core::option::Option<&mut Vec3> {
                self._has
                    .r#initial_accel_scale_std()
                    .then_some(&mut self.r#initial_accel_scale_std)
            }
            /// Clear the presence of `initial_accel_scale_std`
            #[inline]
            pub fn clear_initial_accel_scale_std(&mut self) -> &mut Self {
                self._has.clear_initial_accel_scale_std();
                self
            }
            /// Take the value of `initial_accel_scale_std` and clear its presence
            #[inline]
            pub fn take_initial_accel_scale_std(
                &mut self,
            ) -> ::core::option::Option<Vec3> {
                let val = self
                    ._has
                    .r#initial_accel_scale_std()
                    .then(|| ::core::mem::take(&mut self.r#initial_accel_scale_std));
                self._has.clear_initial_accel_scale_std();
                val
            }
            /// Builder method that sets the value of `initial_accel_scale_std`. Useful for initializing the message.
            #[inline]
            pub fn init_initial_accel_scale_std(mut self, value: Vec3) -> Self {
                self.set_initial_accel_scale_std(value);
                self
            }
            /// Return a reference to `imu_noise` as an `Option`
            #[inline]
            pub fn r#imu_noise(&self) -> ::core::option::Option<&ImuNoise> {
                self._has.r#imu_noise().then_some(&self.r#imu_noise)
            }
            /// Set the value and presence of `imu_noise`
            #[inline]
            pub fn set_imu_noise(&mut self, value: ImuNoise) -> &mut Self {
                self._has.set_imu_noise();
                self.r#imu_noise = value.into();
                self
            }
            /// Return a mutable reference to `imu_noise` as an `Option`
            #[inline]
            pub fn mut_imu_noise(&mut self) -> ::core::option::Option<&mut ImuNoise> {
                self._has.r#imu_noise().then_some(&mut self.r#imu_noise)
            }
            /// Clear the presence of `imu_noise`
            #[inline]
            pub fn clear_imu_noise(&mut self) -> &mut Self {
                self._has.clear_imu_noise();
                self
            }
            /// Take the value of `imu_noise` and clear its presence
            #[inline]
            pub fn take_imu_noise(&mut self) -> ::core::option::Option<ImuNoise> {
                let val = self
                    ._has
                    .r#imu_noise()
                    .then(|| ::core::mem::take(&mut self.r#imu_noise));
                self._has.clear_imu_noise();
                val
            }
            /// Builder method that sets the value of `imu_noise`. Useful for initializing the message.
            #[inline]
            pub fn init_imu_noise(mut self, value: ImuNoise) -> Self {
                self.set_imu_noise(value);
                self
            }
            /// Return a reference to `antenna_lever_arm_m` as an `Option`
            #[inline]
            pub fn r#antenna_lever_arm_m(&self) -> ::core::option::Option<&Vec3> {
                self._has.r#antenna_lever_arm_m().then_some(&self.r#antenna_lever_arm_m)
            }
            /// Set the value and presence of `antenna_lever_arm_m`
            #[inline]
            pub fn set_antenna_lever_arm_m(&mut self, value: Vec3) -> &mut Self {
                self._has.set_antenna_lever_arm_m();
                self.r#antenna_lever_arm_m = value.into();
                self
            }
            /// Return a mutable reference to `antenna_lever_arm_m` as an `Option`
            #[inline]
            pub fn mut_antenna_lever_arm_m(
                &mut self,
            ) -> ::core::option::Option<&mut Vec3> {
                self._has
                    .r#antenna_lever_arm_m()
                    .then_some(&mut self.r#antenna_lever_arm_m)
            }
            /// Clear the presence of `antenna_lever_arm_m`
            #[inline]
            pub fn clear_antenna_lever_arm_m(&mut self) -> &mut Self {
                self._has.clear_antenna_lever_arm_m();
                self
            }
            /// Take the value of `antenna_lever_arm_m` and clear its presence
            #[inline]
            pub fn take_antenna_lever_arm_m(&mut self) -> ::core::option::Option<Vec3> {
                let val = self
                    ._has
                    .r#antenna_lever_arm_m()
                    .then(|| ::core::mem::take(&mut self.r#antenna_lever_arm_m));
                self._has.clear_antenna_lever_arm_m();
                val
            }
            /// Builder method that sets the value of `antenna_lever_arm_m`. Useful for initializing the message.
            #[inline]
            pub fn init_antenna_lever_arm_m(mut self, value: Vec3) -> Self {
                self.set_antenna_lever_arm_m(value);
                self
            }
            /// Return a reference to `max_consecutive_rejections`
            #[inline]
            pub fn r#max_consecutive_rejections(&self) -> &u32 {
                &self.r#max_consecutive_rejections
            }
            /// Return a mutable reference to `max_consecutive_rejections`
            #[inline]
            pub fn mut_max_consecutive_rejections(&mut self) -> &mut u32 {
                &mut self.r#max_consecutive_rejections
            }
            /// Set the value of `max_consecutive_rejections`
            #[inline]
            pub fn set_max_consecutive_rejections(&mut self, value: u32) -> &mut Self {
                self.r#max_consecutive_rejections = value.into();
                self
            }
            /// Builder method that sets the value of `max_consecutive_rejections`. Useful for initializing the message.
            #[inline]
            pub fn init_max_consecutive_rejections(mut self, value: u32) -> Self {
                self.r#max_consecutive_rejections = value.into();
                self
            }
            /// Return a reference to `rejection_inflation`
            #[inline]
            pub fn r#rejection_inflation(&self) -> &f64 {
                &self.r#rejection_inflation
            }
            /// Return a mutable reference to `rejection_inflation`
            #[inline]
            pub fn mut_rejection_inflation(&mut self) -> &mut f64 {
                &mut self.r#rejection_inflation
            }
            /// Set the value of `rejection_inflation`
            #[inline]
            pub fn set_rejection_inflation(&mut self, value: f64) -> &mut Self {
                self.r#rejection_inflation = value.into();
                self
            }
            /// Builder method that sets the value of `rejection_inflation`. Useful for initializing the message.
            #[inline]
            pub fn init_rejection_inflation(mut self, value: f64) -> Self {
                self.r#rejection_inflation = value.into();
                self
            }
        }
        impl ::micropb::MessageDecode for GinsOptions {
            fn decode<IMPL_MICROPB_READ: ::micropb::PbRead>(
                &mut self,
                decoder: &mut ::micropb::PbDecoder<IMPL_MICROPB_READ>,
                len: usize,
            ) -> Result<(), ::micropb::DecodeError<IMPL_MICROPB_READ::Error>> {
                use ::micropb::{PbBytes, PbString, PbVec, PbMap, FieldDecode};
                let before = decoder.bytes_read();
                while decoder.bytes_read() - before < len {
                    let tag = decoder.decode_tag()?;
                    match tag.field_num() {
                        0 => return Err(::micropb::DecodeError::ZeroField),
                        1u32 => {
                            let mut_ref = &mut self.r#initial_position;
                            {
                                mut_ref.decode_len_delimited(decoder)?;
                            };
                            self._has.set_initial_position();
                        }
                        2u32 => {
                            let mut_ref = &mut self.r#initial_velocity_mps;
                            {
                                mut_ref.decode_len_delimited(decoder)?;
                            };
                            self._has.set_initial_velocity_mps();
                        }
                        3u32 => {
                            let mut_ref = &mut self.r#initial_attitude;
                            {
                                mut_ref.decode_len_delimited(decoder)?;
                            };
                            self._has.set_initial_attitude();
                        }
                        4u32 => {
                            let mut_ref = &mut self.r#initial_imu_error;
                            {
                                mut_ref.decode_len_delimited(decoder)?;
                            };
                            self._has.set_initial_imu_error();
                        }
                        5u32 => {
                            let mut_ref = &mut self.r#initial_position_std_m;
                            {
                                mut_ref.decode_len_delimited(decoder)?;
                            };
                            self._has.set_initial_position_std_m();
                        }
                        6u32 => {
                            let mut_ref = &mut self.r#initial_velocity_std_mps;
                            {
                                mut_ref.decode_len_delimited(decoder)?;
                            };
                            self._has.set_initial_velocity_std_mps();
                        }
                        7u32 => {
                            let mut_ref = &mut self.r#initial_attitude_std_rad;
                            {
                                mut_ref.decode_len_delimited(decoder)?;
                            };
                            self._has.set_initial_attitude_std_rad();
                        }
                        8u32 => {
                            let mut_ref = &mut self.r#initial_gyro_bias_std_rps;
                            {
                                mut_ref.decode_len_delimited(decoder)?;
                            };
                            self._has.set_initial_gyro_bias_std_rps();
                        }
                        9u32 => {
                            let mut_ref = &mut self.r#initial_accel_bias_std_mps2;
                            {
                                mut_ref.decode_len_delimited(decoder)?;
                            };
                            self._has.set_initial_accel_bias_std_mps2();
                        }
                        10u32 => {
                            let mut_ref = &mut self.r#initial_gyro_scale_std;
                            {
                                mut_ref.decode_len_delimited(decoder)?;
                            };
                            self._has.set_initial_gyro_scale_std();
                        }
                        11u32 => {
                            let mut_ref = &mut self.r#initial_accel_scale_std;
                            {
                                mut_ref.decode_len_delimited(decoder)?;
                            };
                            self._has.set_initial_accel_scale_std();
                        }
                        12u32 => {
                            let mut_ref = &mut self.r#imu_noise;
                            {
                                mut_ref.decode_len_delimited(decoder)?;
                            };
                            self._has.set_imu_noise();
                        }
                        13u32 => {
                            let mut_ref = &mut self.r#antenna_lever_arm_m;
                            {
                                mut_ref.decode_len_delimited(decoder)?;
                            };
                            self._has.set_antenna_lever_arm_m();
                        }
                        14u32 => {
                            let mut_ref = &mut self.r#max_consecutive_rejections;
                            {
                                let val = decoder.decode_varint32()?;
                                let val_ref = &val;
                                if *val_ref != 0 {
                                    *mut_ref = val as _;
                                }
                            };
                        }
                        15u32 => {
                            let mut_ref = &mut self.r#rejection_inflation;
                            {
                                let val = decoder.decode_double()?;
                                let val_ref = &val;
                                if *val_ref != 0.0 {
                                    *mut_ref = val as _;
                                }
                            };
                        }
                        _ => {
                            decoder.skip_wire_value(tag.wire_type())?;
                        }
                    }
                }
                Ok(())
            }
        }
        impl ::micropb::MessageEncode for GinsOptions {
            const MAX_SIZE: ::core::result::Result<usize, &'static str> = 'msg: {
                let mut max_size = 0;
                match ::micropb::const_map!(
                    ::micropb::const_map!(< Lla as ::micropb::MessageEncode > ::MAX_SIZE,
                    | size | ::micropb::size::sizeof_len_record(size)), | size | size +
                    1usize
                ) {
                    ::core::result::Result::Ok(size) => {
                        max_size += size;
                    }
                    ::core::result::Result::Err(err) => {
                        break 'msg (::core::result::Result::<usize, _>::Err(err));
                    }
                }
                match ::micropb::const_map!(
                    ::micropb::const_map!(< Ned as ::micropb::MessageEncode > ::MAX_SIZE,
                    | size | ::micropb::size::sizeof_len_record(size)), | size | size +
                    1usize
                ) {
                    ::core::result::Result::Ok(size) => {
                        max_size += size;
                    }
                    ::core::result::Result::Err(err) => {
                        break 'msg (::core::result::Result::<usize, _>::Err(err));
                    }
                }
                match ::micropb::const_map!(
                    ::micropb::const_map!(< Euler as ::micropb::MessageEncode >
                    ::MAX_SIZE, | size | ::micropb::size::sizeof_len_record(size)), |
                    size | size + 1usize
                ) {
                    ::core::result::Result::Ok(size) => {
                        max_size += size;
                    }
                    ::core::result::Result::Err(err) => {
                        break 'msg (::core::result::Result::<usize, _>::Err(err));
                    }
                }
                match ::micropb::const_map!(
                    ::micropb::const_map!(< ImuError as ::micropb::MessageEncode >
                    ::MAX_SIZE, | size | ::micropb::size::sizeof_len_record(size)), |
                    size | size + 1usize
                ) {
                    ::core::result::Result::Ok(size) => {
                        max_size += size;
                    }
                    ::core::result::Result::Err(err) => {
                        break 'msg (::core::result::Result::<usize, _>::Err(err));
                    }
                }
                match ::micropb::const_map!(
                    ::micropb::const_map!(< Vec3 as ::micropb::MessageEncode >
                    ::MAX_SIZE, | size | ::micropb::size::sizeof_len_record(size)), |
                    size | size + 1usize
                ) {
                    ::core::result::Result::Ok(size) => {
                        max_size += size;
                    }
                    ::core::result::Result::Err(err) => {
                        break 'msg (::core::result::Result::<usize, _>::Err(err));
                    }
                }
                match ::micropb::const_map!(
                    ::micropb::const_map!(< Vec3 as ::micropb::MessageEncode >
                    ::MAX_SIZE, | size | ::micropb::size::sizeof_len_record(size)), |
                    size | size + 1usize
                ) {
                    ::core::result::Result::Ok(size) => {
                        max_size += size;
                    }
                    ::core::result::Result::Err(err) => {
                        break 'msg (::core::result::Result::<usize, _>::Err(err));
                    }
                }
                match ::micropb::const_map!(
                    ::micropb::const_map!(< Vec3 as ::micropb::MessageEncode >
                    ::MAX_SIZE, | size | ::micropb::size::sizeof_len_record(size)), |
                    size | size + 1usize
                ) {
                    ::core::result::Result::Ok(size) => {
                        max_size += size;
                    }
                    ::core::result::Result::Err(err) => {
                        break 'msg (::core::result::Result::<usize, _>::Err(err));
                    }
                }
                match ::micropb::const_map!(
                    ::micropb::const_map!(< Vec3 as ::micropb::MessageEncode >
                    ::MAX_SIZE, | size | ::micropb::size::sizeof_len_record(size)), |
                    size | size + 1usize
                ) {
                    ::core::result::Result::Ok(size) => {
                        max_size += size;
                    }
                    ::core::result::Result::Err(err) => {
                        break 'msg (::core::result::Result::<usize, _>::Err(err));
                    }
                }
                match ::micropb::const_map!(
                    ::micropb::const_map!(< Vec3 as ::micropb::MessageEncode >
                    ::MAX_SIZE, | size | ::micropb::size::sizeof_len_record(size)), |
                    size | size + 1usize
                ) {
                    ::core::result::Result::Ok(size) => {
                        max_size += size;
                    }
                    ::core::result::Result::Err(err) => {
                        break 'msg (::core::result::Result::<usize, _>::Err(err));
                    }
                }
                match ::micropb::const_map!(
                    ::micropb::const_map!(< Vec3 as ::micropb::MessageEncode >
                    ::MAX_SIZE, | size | ::micropb::size::sizeof_len_record(size)), |
                    size | size + 1usize
                ) {
                    ::core::result::Result::Ok(size) => {
                        max_size += size;
                    }
                    ::core::result::Result::Err(err) => {
                        break 'msg (::core::result::Result::<usize, _>::Err(err));
                    }
                }
                match ::micropb::const_map!(
                    ::micropb::const_map!(< Vec3 as ::micropb::MessageEncode >
                    ::MAX_SIZE, | size | ::micropb::size::sizeof_len_record(size)), |
                    size | size + 1usize
                ) {
                    ::core::result::Result::Ok(size) => {
                        max_size += size;
                    }
                    ::core::result::Result::Err(err) => {
                        break 'msg (::core::result::Result::<usize, _>::Err(err));
                    }
                }
                match ::micropb::const_map!(
                    ::micropb::const_map!(< ImuNoise as ::micropb::MessageEncode >
                    ::MAX_SIZE, | size | ::micropb::size::sizeof_len_record(size)), |
                    size | size + 1usize
                ) {
                    ::core::result::Result::Ok(size) => {
                        max_size += size;
                    }
                    ::core::result::Result::Err(err) => {
                        break 'msg (::core::result::Result::<usize, _>::Err(err));
                    }
                }
                match ::micropb::const_map!(
                    ::micropb::const_map!(< Vec3 as ::micropb::MessageEncode >
                    ::MAX_SIZE, | size | ::micropb::size::sizeof_len_record(size)), |
                    size | size + 1usize
                ) {
                    ::core::result::Result::Ok(size) => {
                        max_size += size;
                    }
                    ::core::result::Result::Err(err) => {
                        break 'msg (::core::result::Result::<usize, _>::Err(err));
                    }
                }
                match ::micropb::const_map!(
                    ::core::result::Result::Ok(5usize), | size | size + 1usize
                ) {
                    ::core::result::Result::Ok(size) => {
                        max_size += size;
                    }
                    ::core::result::Result::Err(err) => {
                        break 'msg (::core::result::Result::<usize, _>::Err(err));
                    }
                }
                match ::micropb::const_map!(
                    ::core::result::Result::Ok(8usize), | size | size + 1usize
                ) {
                    ::core::result::Result::Ok(size) => {
                        max_size += size;
                    }
                    ::core::result::Result::Err(err) => {
                        break 'msg (::core::result::Result::<usize, _>::Err(err));
                    }
                }
                ::core::result::Result::Ok(max_size)
            };
            fn encode<IMPL_MICROPB_WRITE: ::micropb::PbWrite>(
                &self,
                encoder: &mut ::micropb::PbEncoder<IMPL_MICROPB_WRITE>,
            ) -> Result<(), IMPL_MICROPB_WRITE::Error> {
                use ::micropb::{PbMap, FieldEncode};
                {
                    if let ::core::option::Option::Some(val_ref) = self
                        .r#initial_position()
                    {
                        encoder.encode_varint32(10u32)?;
                        val_ref.encode_len_delimited(encoder)?;
                    }
                }
                {
                    if let ::core::option::Option::Some(val_ref) = self
                        .r#initial_velocity_mps()
                    {
                        encoder.encode_varint32(18u32)?;
                        val_ref.encode_len_delimited(encoder)?;
                    }
                }
                {
                    if let ::core::option::Option::Some(val_ref) = self
                        .r#initial_attitude()
                    {
                        encoder.encode_varint32(26u32)?;
                        val_ref.encode_len_delimited(encoder)?;
                    }
                }
                {
                    if let ::core::option::Option::Some(val_ref) = self
                        .r#initial_imu_error()
                    {
                        encoder.encode_varint32(34u32)?;
                        val_ref.encode_len_delimited(encoder)?;
                    }
                }
                {
                    if let ::core::option::Option::Some(val_ref) = self
                        .r#initial_position_std_m()
                    {
                        encoder.encode_varint32(42u32)?;
                        val_ref.encode_len_delimited(encoder)?;
                    }
                }
                {
                    if let ::core::option::Option::Some(val_ref) = self
                        .r#initial_velocity_std_mps()
                    {
                        encoder.encode_varint32(50u32)?;
                        val_ref.encode_len_delimited(encoder)?;
                    }
                }
                {
                    if let ::core::option::Option::Some(val_ref) = self
                        .r#initial_attitude_std_rad()
                    {
                        encoder.encode_varint32(58u32)?;
                        val_ref.encode_len_delimited(encoder)?;
                    }
                }
                {
                    if let ::core::option::Option::Some(val_ref) = self
                        .r#initial_gyro_bias_std_rps()
                    {
                        encoder.encode_varint32(66u32)?;
                        val_ref.encode_len_delimited(encoder)?;
                    }
                }
                {
                    if let ::core::option::Option::Some(val_ref) = self
                        .r#initial_accel_bias_std_mps2()
                    {
                        encoder.encode_varint32(74u32)?;
                        val_ref.encode_len_delimited(encoder)?;
                    }
                }
                {
                    if let ::core::option::Option::Some(val_ref) = self
                        .r#initial_gyro_scale_std()
                    {
                        encoder.encode_varint32(82u32)?;
                        val_ref.encode_len_delimited(encoder)?;
                    }
                }
                {
                    if let ::core::option::Option::Some(val_ref) = self
                        .r#initial_accel_scale_std()
                    {
                        encoder.encode_varint32(90u32)?;
                        val_ref.encode_len_delimited(encoder)?;
                    }
                }
                {
                    if let ::core::option::Option::Some(val_ref) = self.r#imu_noise() {
                        encoder.encode_varint32(98u32)?;
                        val_ref.encode_len_delimited(encoder)?;
                    }
                }
                {
                    if let ::core::option::Option::Some(val_ref) = self
                        .r#antenna_lever_arm_m()
                    {
                        encoder.encode_varint32(106u32)?;
                        val_ref.encode_len_delimited(encoder)?;
                    }
                }
                {
                    let val_ref = &self.r#max_consecutive_rejections;
                    if *val_ref != 0 {
                        encoder.encode_varint32(112u32)?;
                        encoder.encode_varint32(*val_ref as _)?;
                    }
                }
                {
                    let val_ref = &self.r#rejection_inflation;
                    if *val_ref != 0.0 {
                        encoder.encode_varint32(121u32)?;
                        encoder.encode_double(*val_ref)?;
                    }
                }
                Ok(())
            }
            fn compute_size(&self) -> usize {
                use ::micropb::{PbMap, FieldEncode};
                let mut size = 0;
                {
                    if let ::core::option::Option::Some(val_ref) = self
                        .r#initial_position()
                    {
                        size
                            += 1usize
                                + ::micropb::size::sizeof_len_record(
                                    val_ref.compute_size(),
                                );
                    }
                }
                {
                    if let ::core::option::Option::Some(val_ref) = self
                        .r#initial_velocity_mps()
                    {
                        size
                            += 1usize
                                + ::micropb::size::sizeof_len_record(
                                    val_ref.compute_size(),
                                );
                    }
                }
                {
                    if let ::core::option::Option::Some(val_ref) = self
                        .r#initial_attitude()
                    {
                        size
                            += 1usize
                                + ::micropb::size::sizeof_len_record(
                                    val_ref.compute_size(),
                                );
                    }
                }
                {
                    if let ::core::option::Option::Some(val_ref) = self
                        .r#initial_imu_error()
                    {
                        size
                            += 1usize
                                + ::micropb::size::sizeof_len_record(
                                    val_ref.compute_size(),
                                );
                    }
                }
                {
                    if let ::core::option::Option::Some(val_ref) = self
                        .r#initial_position_std_m()
                    {
                        size
                            += 1usize
                                + ::micropb::size::sizeof_len_record(
                                    val_ref.compute_size(),
                                );
                    }
                }
                {
                    if let ::core::option::Option::Some(val_ref) = self
                        .r#initial_velocity_std_mps()
                    {
                        size
                            += 1usize
                                + ::micropb::size::sizeof_len_record(
                                    val_ref.compute_size(),
                                );
                    }
                }
                {
                    if let ::core::option::Option::Some(val_ref) = self
                        .r#initial_attitude_std_rad()
                    {
                        size
                            += 1usize
                                + ::micropb::size::sizeof_len_record(
                                    val_ref.compute_size(),
                                );
                    }
                }
                {
                    if let ::core::option::Option::Some(val_ref) = self
                        .r#initial_gyro_bias_std_rps()
                    {
                        size
                            += 1usize
                                + ::micropb::size::sizeof_len_record(
                                    val_ref.compute_size(),
                                );
                    }
                }
                {
                    if let ::core::option::Option::Some(val_ref) = self
                        .r#initial_accel_bias_std_mps2()
                    {
                        size
                            += 1usize
                                + ::micropb::size::sizeof_len_record(
                                    val_ref.compute_size(),
                                );
                    }
                }
                {
                    if let ::core::option::Option::Some(val_ref) = self
                        .r#initial_gyro_scale_std()
                    {
                        size
                            += 1usize
                                + ::micropb::size::sizeof_len_record(
                                    val_ref.compute_size(),
                                );
                    }
                }
                {
                    if let ::core::option::Option::Some(val_ref) = self
                        .r#initial_accel_scale_std()
                    {
                        size
                            += 1usize
                                + ::micropb::size::sizeof_len_record(
                                    val_ref.compute_size(),
                                );
                    }
                }
                {
                    if let ::core::option::Option::Some(val_ref) = self.r#imu_noise() {
                        size
                            += 1usize
                                + ::micropb::size::sizeof_len_record(
                                    val_ref.compute_size(),
                                );
                    }
                }
                {
                    if let ::core::option::Option::Some(val_ref) = self
                        .r#antenna_lever_arm_m()
                    {
                        size
                            += 1usize
                                + ::micropb::size::sizeof_len_record(
                                    val_ref.compute_size(),
                                );
                    }
                }
                {
                    let val_ref = &self.r#max_consecutive_rejections;
                    if *val_ref != 0 {
                        size += 1usize + ::micropb::size::sizeof_varint32(*val_ref as _);
                    }
                }
                {
                    let val_ref = &self.r#rejection_inflation;
                    if *val_ref != 0.0 {
                        size += 1usize + 8;
                    }
                }
                size
            }
        }
        /// Inner types for `GinsOptions`
        pub mod GinsOptions_ {
            /// Compact bitfield for tracking presence of optional and message fields
            #[derive(Debug, Default, PartialEq, Clone, Copy)]
            pub struct _Hazzer([u8; 2]);
            impl _Hazzer {
                /// New hazzer with all fields set to off
                #[inline]
                pub const fn _new() -> Self {
                    Self([0; 2])
                }
                /// Query presence of `initial_position`
                #[inline]
                pub const fn r#initial_position(&self) -> bool {
                    (self.0[0] & 1) != 0
                }
                /// Set presence of `initial_position`
                #[inline]
                pub const fn set_initial_position(&mut self) -> &mut Self {
                    let elem = &mut self.0[0];
                    *elem |= 1;
                    self
                }
                /// Clear presence of `initial_position`
                #[inline]
                pub const fn clear_initial_position(&mut self) -> &mut Self {
                    let elem = &mut self.0[0];
                    *elem &= !1;
                    self
                }
                /// Builder method that sets the presence of `initial_position`. Useful for initializing the Hazzer.
                #[inline]
                pub const fn init_initial_position(mut self) -> Self {
                    self.set_initial_position();
                    self
                }
                /// Query presence of `initial_velocity_mps`
                #[inline]
                pub const fn r#initial_velocity_mps(&self) -> bool {
                    (self.0[0] & 2) != 0
                }
                /// Set presence of `initial_velocity_mps`
                #[inline]
                pub const fn set_initial_velocity_mps(&mut self) -> &mut Self {
                    let elem = &mut self.0[0];
                    *elem |= 2;
                    self
                }
                /// Clear presence of `initial_velocity_mps`
                #[inline]
                pub const fn clear_initial_velocity_mps(&mut self) -> &mut Self {
                    let elem = &mut self.0[0];
                    *elem &= !2;
                    self
                }
                /// Builder method that sets the presence of `initial_velocity_mps`. Useful for initializing the Hazzer.
                #[inline]
                pub const fn init_initial_velocity_mps(mut self) -> Self {
                    self.set_initial_velocity_mps();
                    self
                }
                /// Query presence of `initial_attitude`
                #[inline]
                pub const fn r#initial_attitude(&self) -> bool {
                    (self.0[0] & 4) != 0
                }
                /// Set presence of `initial_attitude`
                #[inline]
                pub const fn set_initial_attitude(&mut self) -> &mut Self {
                    let elem = &mut self.0[0];
                    *elem |= 4;
                    self
                }
                /// Clear presence of `initial_attitude`
                #[inline]
                pub const fn clear_initial_attitude(&mut self) -> &mut Self {
                    let elem = &mut self.0[0];
                    *elem &= !4;
                    self
                }
                /// Builder method that sets the presence of `initial_attitude`. Useful for initializing the Hazzer.
                #[inline]
                pub const fn init_initial_attitude(mut self) -> Self {
                    self.set_initial_attitude();
                    self
                }
                /// Query presence of `initial_imu_error`
                #[inline]
                pub const fn r#initial_imu_error(&self) -> bool {
                    (self.0[0] & 8) != 0
                }
                /// Set presence of `initial_imu_error`
                #[inline]
                pub const fn set_initial_imu_error(&mut self) -> &mut Self {
                    let elem = &mut self.0[0];
                    *elem |= 8;
                    self
                }
                /// Clear presence of `initial_imu_error`
                #[inline]
                pub const fn clear_initial_imu_error(&mut self) -> &mut Self {
                    let elem = &mut self.0[0];
                    *elem &= !8;
                    self
                }
                /// Builder method that sets the presence of `initial_imu_error`. Useful for initializing the Hazzer.
                #[inline]
                pub const fn init_initial_imu_error(mut self) -> Self {
                    self.set_initial_imu_error();
                    self
                }
                /// Query presence of `initial_position_std_m`
                #[inline]
                pub const fn r#initial_position_std_m(&self) -> bool {
                    (self.0[0] & 16) != 0
                }
                /// Set presence of `initial_position_std_m`
                #[inline]
                pub const fn set_initial_position_std_m(&mut self) -> &mut Self {
                    let elem = &mut self.0[0];
                    *elem |= 16;
                    self
                }
                /// Clear presence of `initial_position_std_m`
                #[inline]
                pub const fn clear_initial_position_std_m(&mut self) -> &mut Self {
                    let elem = &mut self.0[0];
                    *elem &= !16;
                    self
                }
                /// Builder method that sets the presence of `initial_position_std_m`. Useful for initializing the Hazzer.
                #[inline]
                pub const fn init_initial_position_std_m(mut self) -> Self {
                    self.set_initial_position_std_m();
                    self
                }
                /// Query presence of `initial_velocity_std_mps`
                #[inline]
                pub const fn r#initial_velocity_std_mps(&self) -> bool {
                    (self.0[0] & 32) != 0
                }
                /// Set presence of `initial_velocity_std_mps`
                #[inline]
                pub const fn set_initial_velocity_std_mps(&mut self) -> &mut Self {
                    let elem = &mut self.0[0];
                    *elem |= 32;
                    self
                }
                /// Clear presence of `initial_velocity_std_mps`
                #[inline]
                pub const fn clear_initial_velocity_std_mps(&mut self) -> &mut Self {
                    let elem = &mut self.0[0];
                    *elem &= !32;
                    self
                }
                /// Builder method that sets the presence of `initial_velocity_std_mps`. Useful for initializing the Hazzer.
                #[inline]
                pub const fn init_initial_velocity_std_mps(mut self) -> Self {
                    self.set_initial_velocity_std_mps();
                    self
                }
                /// Query presence of `initial_attitude_std_rad`
                #[inline]
                pub const fn r#initial_attitude_std_rad(&self) -> bool {
                    (self.0[0] & 64) != 0
                }
                /// Set presence of `initial_attitude_std_rad`
                #[inline]
                pub const fn set_initial_attitude_std_rad(&mut self) -> &mut Self {
                    let elem = &mut self.0[0];
                    *elem |= 64;
                    self
                }
                /// Clear presence of `initial_attitude_std_rad`
                #[inline]
                pub const fn clear_initial_attitude_std_rad(&mut self) -> &mut Self {
                    let elem = &mut self.0[0];
                    *elem &= !64;
                    self
                }
                /// Builder method that sets the presence of `initial_attitude_std_rad`. Useful for initializing the Hazzer.
                #[inline]
                pub const fn init_initial_attitude_std_rad(mut self) -> Self {
                    self.set_initial_attitude_std_rad();
                    self
                }
                /// Query presence of `initial_gyro_bias_std_rps`
                #[inline]
                pub const fn r#initial_gyro_bias_std_rps(&self) -> bool {
                    (self.0[0] & 128) != 0
                }
                /// Set presence of `initial_gyro_bias_std_rps`
                #[inline]
                pub const fn set_initial_gyro_bias_std_rps(&mut self) -> &mut Self {
                    let elem = &mut self.0[0];
                    *elem |= 128;
                    self
                }
                /// Clear presence of `initial_gyro_bias_std_rps`
                #[inline]
                pub const fn clear_initial_gyro_bias_std_rps(&mut self) -> &mut Self {
                    let elem = &mut self.0[0];
                    *elem &= !128;
                    self
                }
                /// Builder method that sets the presence of `initial_gyro_bias_std_rps`. Useful for initializing the Hazzer.
                #[inline]
                pub const fn init_initial_gyro_bias_std_rps(mut self) -> Self {
                    self.set_initial_gyro_bias_std_rps();
                    self
                }
                /// Query presence of `initial_accel_bias_std_mps2`
                #[inline]
                pub const fn r#initial_accel_bias_std_mps2(&self) -> bool {
                    (self.0[1] & 1) != 0
                }
                /// Set presence of `initial_accel_bias_std_mps2`
                #[inline]
                pub const fn set_initial_accel_bias_std_mps2(&mut self) -> &mut Self {
                    let elem = &mut self.0[1];
                    *elem |= 1;
                    self
                }
                /// Clear presence of `initial_accel_bias_std_mps2`
                #[inline]
                pub const fn clear_initial_accel_bias_std_mps2(&mut self) -> &mut Self {
                    let elem = &mut self.0[1];
                    *elem &= !1;
                    self
                }
                /// Builder method that sets the presence of `initial_accel_bias_std_mps2`. Useful for initializing the Hazzer.
                #[inline]
                pub const fn init_initial_accel_bias_std_mps2(mut self) -> Self {
                    self.set_initial_accel_bias_std_mps2();
                    self
                }
                /// Query presence of `initial_gyro_scale_std`
                #[inline]
                pub const fn r#initial_gyro_scale_std(&self) -> bool {
                    (self.0[1] & 2) != 0
                }
                /// Set presence of `initial_gyro_scale_std`
                #[inline]
                pub const fn set_initial_gyro_scale_std(&mut self) -> &mut Self {
                    let elem = &mut self.0[1];
                    *elem |= 2;
                    self
                }
                /// Clear presence of `initial_gyro_scale_std`
                #[inline]
                pub const fn clear_initial_gyro_scale_std(&mut self) -> &mut Self {
                    let elem = &mut self.0[1];
                    *elem &= !2;
                    self
                }
                /// Builder method that sets the presence of `initial_gyro_scale_std`. Useful for initializing the Hazzer.
                #[inline]
                pub const fn init_initial_gyro_scale_std(mut self) -> Self {
                    self.set_initial_gyro_scale_std();
                    self
                }
                /// Query presence of `initial_accel_scale_std`
                #[inline]
                pub const fn r#initial_accel_scale_std(&self) -> bool {
                    (self.0[1] & 4) != 0
                }
                /// Set presence of `initial_accel_scale_std`
                #[inline]
                pub const fn set_initial_accel_scale_std(&mut self) -> &mut Self {
                    let elem = &mut self.0[1];
                    *elem |= 4;
                    self
                }
                /// Clear presence of `initial_accel_scale_std`
                #[inline]
                pub const fn clear_initial_accel_scale_std(&mut self) -> &mut Self {
                    let elem = &mut self.0[1];
                    *elem &= !4;
                    self
                }
                /// Builder method that sets the presence of `initial_accel_scale_std`. Useful for initializing the Hazzer.
                #[inline]
                pub const fn init_initial_accel_scale_std(mut self) -> Self {
                    self.set_initial_accel_scale_std();
                    self
                }
                /// Query presence of `imu_noise`
                #[inline]
                pub const fn r#imu_noise(&self) -> bool {
                    (self.0[1] & 8) != 0
                }
                /// Set presence of `imu_noise`
                #[inline]
                pub const fn set_imu_noise(&mut self) -> &mut Self {
                    let elem = &mut self.0[1];
                    *elem |= 8;
                    self
                }
                /// Clear presence of `imu_noise`
                #[inline]
                pub const fn clear_imu_noise(&mut self) -> &mut Self {
                    let elem = &mut self.0[1];
                    *elem &= !8;
                    self
                }
                /// Builder method that sets the presence of `imu_noise`. Useful for initializing the Hazzer.
                #[inline]
                pub const fn init_imu_noise(mut self) -> Self {
                    self.set_imu_noise();
                    self
                }
                /// Query presence of `antenna_lever_arm_m`
                #[inline]
                pub const fn r#antenna_lever_arm_m(&self) -> bool {
                    (self.0[1] & 16) != 0
                }
                /// Set presence of `antenna_lever_arm_m`
                #[inline]
                pub const fn set_antenna_lever_arm_m(&mut self) -> &mut Self {
                    let elem = &mut self.0[1];
                    *elem |= 16;
                    self
                }
                /// Clear presence of `antenna_lever_arm_m`
                #[inline]
                pub const fn clear_antenna_lever_arm_m(&mut self) -> &mut Self {
                    let elem = &mut self.0[1];
                    *elem &= !16;
                    self
                }
                /// Builder method that sets the presence of `antenna_lever_arm_m`. Useful for initializing the Hazzer.
                #[inline]
                pub const fn init_antenna_lever_arm_m(mut self) -> Self {
                    self.set_antenna_lever_arm_m();
                    self
                }
            }
        }
    }
}
