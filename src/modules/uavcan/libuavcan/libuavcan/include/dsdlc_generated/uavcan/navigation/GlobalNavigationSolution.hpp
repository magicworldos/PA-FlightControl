/*
 * UAVCAN data structure definition for libuavcan.
 *
 * Autogenerated, do not edit.
 *
 * Source file: /data/temp/PA-FlightControl/src/modules/uavcan/libuavcan/dsdl/uavcan/navigation/2000.GlobalNavigationSolution.uavcan
 */

#ifndef UAVCAN_NAVIGATION_GLOBALNAVIGATIONSOLUTION_HPP_INCLUDED
#define UAVCAN_NAVIGATION_GLOBALNAVIGATIONSOLUTION_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/node/global_data_type_registry.hpp>
#include <uavcan/marshal/types.hpp>

#include <uavcan/Timestamp.hpp>

/******************************* Source text **********************************
#
# Inertial data and orientation in body frame with fused location.
#
# Fields marked as optional should be set to NaN if the corresponding value is unknown.
#

#
# Global network synchronized timestamp, if known.
# Set to zero if the timestamp is not known.
#
uavcan.Timestamp timestamp

#
# Geo location [angular degree].
#
float64 longitude                   # required
float64 latitude                    # required

#
# Height estimates [meter].
#
float32 height_ellipsoid            # Above ellipsoid (required)
float32 height_msl                  # Above the mean sea level (required)
float32 height_agl                  # Above ground level (provided by radar altimeter or LIDAR) (optional)
float32 height_baro                 # Barometric height (optional)

#
# Atmospheric pressure adjusted to sea level [hectopascal].
#
float16 qnh_hpa                     # optional

#
# Rotation quaternion between the NED frame and the body frame.
# Zero rotation corresponds to the following orientation:
#   X facing north
#   Y facing east
#   Z facing down
#
float32[4] orientation_xyzw

#
# Column order:
#   longitude                                   [meter^2]
#   latitude                                    [meter^2]
#   height (MSL or ellipsoid, whichever worse)  [meter^2]
#   roll angle                                  [radian^2]
#   pitch angle                                 [radian^2]
#   yaw angle                                   [radian^2]
#
float16[<=36] pose_covariance

#
# Linear velocity in the body frame, X-Y-Z [meter/second].
#
float32[3] linear_velocity_body

#
# Angular velocity in the body frame, roll-pitch-yaw [radian/second].
#
float32[3] angular_velocity_body

#
# Low resolution estimate of the linear acceleration in the body frame [(meter/second)^2].
# This estimate should be properly downsampled in order to avoid aliasing effects.
#
float16[3] linear_acceleration_body

#
# Column order:
#   X velocity      [(meter/second)^2]
#   Y velocity      [(meter/second)^2]
#   Z velocity      [(meter/second)^2]
#   roll velocity   [(radian/second)^2]
#   pitch velocity  [(radian/second)^2]
#   yaw velocity    [(radian/second)^2]
#
float16[<=36] velocity_covariance
******************************************************************************/

/********************* DSDL signature source definition ***********************
uavcan.navigation.GlobalNavigationSolution
uavcan.Timestamp timestamp
saturated float64 longitude
saturated float64 latitude
saturated float32 height_ellipsoid
saturated float32 height_msl
saturated float32 height_agl
saturated float32 height_baro
saturated float16 qnh_hpa
saturated float32[4] orientation_xyzw
saturated float16[<=36] pose_covariance
saturated float32[3] linear_velocity_body
saturated float32[3] angular_velocity_body
saturated float16[3] linear_acceleration_body
saturated float16[<=36] velocity_covariance
******************************************************************************/

#undef timestamp
#undef longitude
#undef latitude
#undef height_ellipsoid
#undef height_msl
#undef height_agl
#undef height_baro
#undef qnh_hpa
#undef orientation_xyzw
#undef pose_covariance
#undef linear_velocity_body
#undef angular_velocity_body
#undef linear_acceleration_body
#undef velocity_covariance

namespace uavcan
{
namespace navigation
{

template <int _tmpl>
struct UAVCAN_EXPORT GlobalNavigationSolution_
{
    typedef const GlobalNavigationSolution_<_tmpl>& ParameterType;
    typedef GlobalNavigationSolution_<_tmpl>& ReferenceType;

    struct ConstantTypes
    {
    };

    struct FieldTypes
    {
        typedef ::uavcan::Timestamp timestamp;
        typedef ::uavcan::FloatSpec< 64, ::uavcan::CastModeSaturate > longitude;
        typedef ::uavcan::FloatSpec< 64, ::uavcan::CastModeSaturate > latitude;
        typedef ::uavcan::FloatSpec< 32, ::uavcan::CastModeSaturate > height_ellipsoid;
        typedef ::uavcan::FloatSpec< 32, ::uavcan::CastModeSaturate > height_msl;
        typedef ::uavcan::FloatSpec< 32, ::uavcan::CastModeSaturate > height_agl;
        typedef ::uavcan::FloatSpec< 32, ::uavcan::CastModeSaturate > height_baro;
        typedef ::uavcan::FloatSpec< 16, ::uavcan::CastModeSaturate > qnh_hpa;
        typedef ::uavcan::Array< ::uavcan::FloatSpec< 32, ::uavcan::CastModeSaturate >, ::uavcan::ArrayModeStatic, 4 > orientation_xyzw;
        typedef ::uavcan::Array< ::uavcan::FloatSpec< 16, ::uavcan::CastModeSaturate >, ::uavcan::ArrayModeDynamic, 36 > pose_covariance;
        typedef ::uavcan::Array< ::uavcan::FloatSpec< 32, ::uavcan::CastModeSaturate >, ::uavcan::ArrayModeStatic, 3 > linear_velocity_body;
        typedef ::uavcan::Array< ::uavcan::FloatSpec< 32, ::uavcan::CastModeSaturate >, ::uavcan::ArrayModeStatic, 3 > angular_velocity_body;
        typedef ::uavcan::Array< ::uavcan::FloatSpec< 16, ::uavcan::CastModeSaturate >, ::uavcan::ArrayModeStatic, 3 > linear_acceleration_body;
        typedef ::uavcan::Array< ::uavcan::FloatSpec< 16, ::uavcan::CastModeSaturate >, ::uavcan::ArrayModeDynamic, 36 > velocity_covariance;
    };

    enum
    {
        MinBitLen
            = FieldTypes::timestamp::MinBitLen
            + FieldTypes::longitude::MinBitLen
            + FieldTypes::latitude::MinBitLen
            + FieldTypes::height_ellipsoid::MinBitLen
            + FieldTypes::height_msl::MinBitLen
            + FieldTypes::height_agl::MinBitLen
            + FieldTypes::height_baro::MinBitLen
            + FieldTypes::qnh_hpa::MinBitLen
            + FieldTypes::orientation_xyzw::MinBitLen
            + FieldTypes::pose_covariance::MinBitLen
            + FieldTypes::linear_velocity_body::MinBitLen
            + FieldTypes::angular_velocity_body::MinBitLen
            + FieldTypes::linear_acceleration_body::MinBitLen
            + FieldTypes::velocity_covariance::MinBitLen
    };

    enum
    {
        MaxBitLen
            = FieldTypes::timestamp::MaxBitLen
            + FieldTypes::longitude::MaxBitLen
            + FieldTypes::latitude::MaxBitLen
            + FieldTypes::height_ellipsoid::MaxBitLen
            + FieldTypes::height_msl::MaxBitLen
            + FieldTypes::height_agl::MaxBitLen
            + FieldTypes::height_baro::MaxBitLen
            + FieldTypes::qnh_hpa::MaxBitLen
            + FieldTypes::orientation_xyzw::MaxBitLen
            + FieldTypes::pose_covariance::MaxBitLen
            + FieldTypes::linear_velocity_body::MaxBitLen
            + FieldTypes::angular_velocity_body::MaxBitLen
            + FieldTypes::linear_acceleration_body::MaxBitLen
            + FieldTypes::velocity_covariance::MaxBitLen
    };

    // Constants

    // Fields
    typename ::uavcan::StorageType< typename FieldTypes::timestamp >::Type timestamp;
    typename ::uavcan::StorageType< typename FieldTypes::longitude >::Type longitude;
    typename ::uavcan::StorageType< typename FieldTypes::latitude >::Type latitude;
    typename ::uavcan::StorageType< typename FieldTypes::height_ellipsoid >::Type height_ellipsoid;
    typename ::uavcan::StorageType< typename FieldTypes::height_msl >::Type height_msl;
    typename ::uavcan::StorageType< typename FieldTypes::height_agl >::Type height_agl;
    typename ::uavcan::StorageType< typename FieldTypes::height_baro >::Type height_baro;
    typename ::uavcan::StorageType< typename FieldTypes::qnh_hpa >::Type qnh_hpa;
    typename ::uavcan::StorageType< typename FieldTypes::orientation_xyzw >::Type orientation_xyzw;
    typename ::uavcan::StorageType< typename FieldTypes::pose_covariance >::Type pose_covariance;
    typename ::uavcan::StorageType< typename FieldTypes::linear_velocity_body >::Type linear_velocity_body;
    typename ::uavcan::StorageType< typename FieldTypes::angular_velocity_body >::Type angular_velocity_body;
    typename ::uavcan::StorageType< typename FieldTypes::linear_acceleration_body >::Type linear_acceleration_body;
    typename ::uavcan::StorageType< typename FieldTypes::velocity_covariance >::Type velocity_covariance;

    GlobalNavigationSolution_()
        : timestamp()
        , longitude()
        , latitude()
        , height_ellipsoid()
        , height_msl()
        , height_agl()
        , height_baro()
        , qnh_hpa()
        , orientation_xyzw()
        , pose_covariance()
        , linear_velocity_body()
        , angular_velocity_body()
        , linear_acceleration_body()
        , velocity_covariance()
    {
        ::uavcan::StaticAssert<_tmpl == 0>::check();  // Usage check

#if UAVCAN_DEBUG
        /*
         * Cross-checking MaxBitLen provided by the DSDL compiler.
         * This check shall never be performed in user code because MaxBitLen value
         * actually depends on the nested types, thus it is not invariant.
         */
        ::uavcan::StaticAssert<1860 == MaxBitLen>::check();
#endif
    }

    bool operator==(ParameterType rhs) const;
    bool operator!=(ParameterType rhs) const { return !operator==(rhs); }

    /**
     * This comparison is based on @ref uavcan::areClose(), which ensures proper comparison of
     * floating point fields at any depth.
     */
    bool isClose(ParameterType rhs) const;

    static int encode(ParameterType self, ::uavcan::ScalarCodec& codec,
                      ::uavcan::TailArrayOptimizationMode tao_mode = ::uavcan::TailArrayOptEnabled);

    static int decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
                      ::uavcan::TailArrayOptimizationMode tao_mode = ::uavcan::TailArrayOptEnabled);

    /*
     * Static type info
     */
    enum { DataTypeKind = ::uavcan::DataTypeKindMessage };
    enum { DefaultDataTypeID = 2000 };

    static const char* getDataTypeFullName()
    {
        return "uavcan.navigation.GlobalNavigationSolution";
    }

    static void extendDataTypeSignature(::uavcan::DataTypeSignature& signature)
    {
        signature.extend(getDataTypeSignature());
    }

    static ::uavcan::DataTypeSignature getDataTypeSignature();

};

/*
 * Out of line struct method definitions
 */

template <int _tmpl>
bool GlobalNavigationSolution_<_tmpl>::operator==(ParameterType rhs) const
{
    return
        timestamp == rhs.timestamp &&
        longitude == rhs.longitude &&
        latitude == rhs.latitude &&
        height_ellipsoid == rhs.height_ellipsoid &&
        height_msl == rhs.height_msl &&
        height_agl == rhs.height_agl &&
        height_baro == rhs.height_baro &&
        qnh_hpa == rhs.qnh_hpa &&
        orientation_xyzw == rhs.orientation_xyzw &&
        pose_covariance == rhs.pose_covariance &&
        linear_velocity_body == rhs.linear_velocity_body &&
        angular_velocity_body == rhs.angular_velocity_body &&
        linear_acceleration_body == rhs.linear_acceleration_body &&
        velocity_covariance == rhs.velocity_covariance;
}

template <int _tmpl>
bool GlobalNavigationSolution_<_tmpl>::isClose(ParameterType rhs) const
{
    return
        ::uavcan::areClose(timestamp, rhs.timestamp) &&
        ::uavcan::areClose(longitude, rhs.longitude) &&
        ::uavcan::areClose(latitude, rhs.latitude) &&
        ::uavcan::areClose(height_ellipsoid, rhs.height_ellipsoid) &&
        ::uavcan::areClose(height_msl, rhs.height_msl) &&
        ::uavcan::areClose(height_agl, rhs.height_agl) &&
        ::uavcan::areClose(height_baro, rhs.height_baro) &&
        ::uavcan::areClose(qnh_hpa, rhs.qnh_hpa) &&
        ::uavcan::areClose(orientation_xyzw, rhs.orientation_xyzw) &&
        ::uavcan::areClose(pose_covariance, rhs.pose_covariance) &&
        ::uavcan::areClose(linear_velocity_body, rhs.linear_velocity_body) &&
        ::uavcan::areClose(angular_velocity_body, rhs.angular_velocity_body) &&
        ::uavcan::areClose(linear_acceleration_body, rhs.linear_acceleration_body) &&
        ::uavcan::areClose(velocity_covariance, rhs.velocity_covariance);
}

template <int _tmpl>
int GlobalNavigationSolution_<_tmpl>::encode(ParameterType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::timestamp::encode(self.timestamp, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::longitude::encode(self.longitude, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::latitude::encode(self.latitude, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::height_ellipsoid::encode(self.height_ellipsoid, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::height_msl::encode(self.height_msl, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::height_agl::encode(self.height_agl, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::height_baro::encode(self.height_baro, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::qnh_hpa::encode(self.qnh_hpa, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::orientation_xyzw::encode(self.orientation_xyzw, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::pose_covariance::encode(self.pose_covariance, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::linear_velocity_body::encode(self.linear_velocity_body, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::angular_velocity_body::encode(self.angular_velocity_body, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::linear_acceleration_body::encode(self.linear_acceleration_body, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::velocity_covariance::encode(self.velocity_covariance, codec,  tao_mode);
    return res;
}

template <int _tmpl>
int GlobalNavigationSolution_<_tmpl>::decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::timestamp::decode(self.timestamp, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::longitude::decode(self.longitude, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::latitude::decode(self.latitude, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::height_ellipsoid::decode(self.height_ellipsoid, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::height_msl::decode(self.height_msl, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::height_agl::decode(self.height_agl, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::height_baro::decode(self.height_baro, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::qnh_hpa::decode(self.qnh_hpa, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::orientation_xyzw::decode(self.orientation_xyzw, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::pose_covariance::decode(self.pose_covariance, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::linear_velocity_body::decode(self.linear_velocity_body, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::angular_velocity_body::decode(self.angular_velocity_body, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::linear_acceleration_body::decode(self.linear_acceleration_body, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::velocity_covariance::decode(self.velocity_covariance, codec,  tao_mode);
    return res;
}

/*
 * Out of line type method definitions
 */
template <int _tmpl>
::uavcan::DataTypeSignature GlobalNavigationSolution_<_tmpl>::getDataTypeSignature()
{
    ::uavcan::DataTypeSignature signature(0x3867B6394A0C0FFCULL);

    FieldTypes::timestamp::extendDataTypeSignature(signature);
    FieldTypes::longitude::extendDataTypeSignature(signature);
    FieldTypes::latitude::extendDataTypeSignature(signature);
    FieldTypes::height_ellipsoid::extendDataTypeSignature(signature);
    FieldTypes::height_msl::extendDataTypeSignature(signature);
    FieldTypes::height_agl::extendDataTypeSignature(signature);
    FieldTypes::height_baro::extendDataTypeSignature(signature);
    FieldTypes::qnh_hpa::extendDataTypeSignature(signature);
    FieldTypes::orientation_xyzw::extendDataTypeSignature(signature);
    FieldTypes::pose_covariance::extendDataTypeSignature(signature);
    FieldTypes::linear_velocity_body::extendDataTypeSignature(signature);
    FieldTypes::angular_velocity_body::extendDataTypeSignature(signature);
    FieldTypes::linear_acceleration_body::extendDataTypeSignature(signature);
    FieldTypes::velocity_covariance::extendDataTypeSignature(signature);

    return signature;
}

/*
 * Out of line constant definitions
 */

/*
 * Final typedef
 */
typedef GlobalNavigationSolution_<0> GlobalNavigationSolution;

namespace
{

const ::uavcan::DefaultDataTypeRegistrator< ::uavcan::navigation::GlobalNavigationSolution > _uavcan_gdtr_registrator_GlobalNavigationSolution;

}

} // Namespace navigation
} // Namespace uavcan

/*
 * YAML streamer specialization
 */
namespace uavcan
{

template <>
class UAVCAN_EXPORT YamlStreamer< ::uavcan::navigation::GlobalNavigationSolution >
{
public:
    template <typename Stream>
    static void stream(Stream& s, ::uavcan::navigation::GlobalNavigationSolution::ParameterType obj, const int level);
};

template <typename Stream>
void YamlStreamer< ::uavcan::navigation::GlobalNavigationSolution >::stream(Stream& s, ::uavcan::navigation::GlobalNavigationSolution::ParameterType obj, const int level)
{
    (void)s;
    (void)obj;
    (void)level;
    if (level > 0)
    {
        s << '\n';
        for (int pos = 0; pos < level; pos++)
        {
            s << "  ";
        }
    }
    s << "timestamp: ";
    YamlStreamer< ::uavcan::navigation::GlobalNavigationSolution::FieldTypes::timestamp >::stream(s, obj.timestamp, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "longitude: ";
    YamlStreamer< ::uavcan::navigation::GlobalNavigationSolution::FieldTypes::longitude >::stream(s, obj.longitude, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "latitude: ";
    YamlStreamer< ::uavcan::navigation::GlobalNavigationSolution::FieldTypes::latitude >::stream(s, obj.latitude, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "height_ellipsoid: ";
    YamlStreamer< ::uavcan::navigation::GlobalNavigationSolution::FieldTypes::height_ellipsoid >::stream(s, obj.height_ellipsoid, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "height_msl: ";
    YamlStreamer< ::uavcan::navigation::GlobalNavigationSolution::FieldTypes::height_msl >::stream(s, obj.height_msl, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "height_agl: ";
    YamlStreamer< ::uavcan::navigation::GlobalNavigationSolution::FieldTypes::height_agl >::stream(s, obj.height_agl, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "height_baro: ";
    YamlStreamer< ::uavcan::navigation::GlobalNavigationSolution::FieldTypes::height_baro >::stream(s, obj.height_baro, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "qnh_hpa: ";
    YamlStreamer< ::uavcan::navigation::GlobalNavigationSolution::FieldTypes::qnh_hpa >::stream(s, obj.qnh_hpa, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "orientation_xyzw: ";
    YamlStreamer< ::uavcan::navigation::GlobalNavigationSolution::FieldTypes::orientation_xyzw >::stream(s, obj.orientation_xyzw, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "pose_covariance: ";
    YamlStreamer< ::uavcan::navigation::GlobalNavigationSolution::FieldTypes::pose_covariance >::stream(s, obj.pose_covariance, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "linear_velocity_body: ";
    YamlStreamer< ::uavcan::navigation::GlobalNavigationSolution::FieldTypes::linear_velocity_body >::stream(s, obj.linear_velocity_body, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "angular_velocity_body: ";
    YamlStreamer< ::uavcan::navigation::GlobalNavigationSolution::FieldTypes::angular_velocity_body >::stream(s, obj.angular_velocity_body, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "linear_acceleration_body: ";
    YamlStreamer< ::uavcan::navigation::GlobalNavigationSolution::FieldTypes::linear_acceleration_body >::stream(s, obj.linear_acceleration_body, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "velocity_covariance: ";
    YamlStreamer< ::uavcan::navigation::GlobalNavigationSolution::FieldTypes::velocity_covariance >::stream(s, obj.velocity_covariance, level + 1);
}

}

namespace uavcan
{
namespace navigation
{

template <typename Stream>
inline Stream& operator<<(Stream& s, ::uavcan::navigation::GlobalNavigationSolution::ParameterType obj)
{
    ::uavcan::YamlStreamer< ::uavcan::navigation::GlobalNavigationSolution >::stream(s, obj, 0);
    return s;
}

} // Namespace navigation
} // Namespace uavcan

#endif // UAVCAN_NAVIGATION_GLOBALNAVIGATIONSOLUTION_HPP_INCLUDED