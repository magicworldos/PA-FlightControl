/*
 * UAVCAN data structure definition for libuavcan.
 *
 * Autogenerated, do not edit.
 *
 * Source file: /data/temp/PA-FlightControl/src/modules/uavcan/libuavcan/dsdl/uavcan/protocol/4.GlobalTimeSync.uavcan
 */

#ifndef UAVCAN_PROTOCOL_GLOBALTIMESYNC_HPP_INCLUDED
#define UAVCAN_PROTOCOL_GLOBALTIMESYNC_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/node/global_data_type_registry.hpp>
#include <uavcan/marshal/types.hpp>

/******************************* Source text **********************************
#
# Global time synchronization.
# Any node that publishes timestamped data must use this time reference.
#
# Please refer to the specification to learn about the synchronization algorithm.
#

#
# Broadcasting period must be within this range.
#
uint16 MAX_BROADCASTING_PERIOD_MS = 1100            # Milliseconds
uint16 MIN_BROADCASTING_PERIOD_MS = 40              # Milliseconds

#
# Synchronization slaves may switch to a new source if the current master was silent for this amount of time.
#
uint16 RECOMMENDED_BROADCASTER_TIMEOUT_MS = 2200    # Milliseconds

#
# Time in microseconds when the PREVIOUS GlobalTimeSync message was transmitted.
# If this message is the first one, this field must be zero.
#
truncated uint56 previous_transmission_timestamp_usec # Microseconds
******************************************************************************/

/********************* DSDL signature source definition ***********************
uavcan.protocol.GlobalTimeSync
truncated uint56 previous_transmission_timestamp_usec
******************************************************************************/

#undef previous_transmission_timestamp_usec
#undef MAX_BROADCASTING_PERIOD_MS
#undef MIN_BROADCASTING_PERIOD_MS
#undef RECOMMENDED_BROADCASTER_TIMEOUT_MS

namespace uavcan
{
namespace protocol
{

template <int _tmpl>
struct UAVCAN_EXPORT GlobalTimeSync_
{
    typedef const GlobalTimeSync_<_tmpl>& ParameterType;
    typedef GlobalTimeSync_<_tmpl>& ReferenceType;

    struct ConstantTypes
    {
        typedef ::uavcan::IntegerSpec< 16, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > MAX_BROADCASTING_PERIOD_MS;
        typedef ::uavcan::IntegerSpec< 16, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > MIN_BROADCASTING_PERIOD_MS;
        typedef ::uavcan::IntegerSpec< 16, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > RECOMMENDED_BROADCASTER_TIMEOUT_MS;
    };

    struct FieldTypes
    {
        typedef ::uavcan::IntegerSpec< 56, ::uavcan::SignednessUnsigned, ::uavcan::CastModeTruncate > previous_transmission_timestamp_usec;
    };

    enum
    {
        MinBitLen
            = FieldTypes::previous_transmission_timestamp_usec::MinBitLen
    };

    enum
    {
        MaxBitLen
            = FieldTypes::previous_transmission_timestamp_usec::MaxBitLen
    };

    // Constants
    static const typename ::uavcan::StorageType< typename ConstantTypes::MAX_BROADCASTING_PERIOD_MS >::Type MAX_BROADCASTING_PERIOD_MS; // 1100
    static const typename ::uavcan::StorageType< typename ConstantTypes::MIN_BROADCASTING_PERIOD_MS >::Type MIN_BROADCASTING_PERIOD_MS; // 40
    static const typename ::uavcan::StorageType< typename ConstantTypes::RECOMMENDED_BROADCASTER_TIMEOUT_MS >::Type RECOMMENDED_BROADCASTER_TIMEOUT_MS; // 2200

    // Fields
    typename ::uavcan::StorageType< typename FieldTypes::previous_transmission_timestamp_usec >::Type previous_transmission_timestamp_usec;

    GlobalTimeSync_()
        : previous_transmission_timestamp_usec()
    {
        ::uavcan::StaticAssert<_tmpl == 0>::check();  // Usage check

#if UAVCAN_DEBUG
        /*
         * Cross-checking MaxBitLen provided by the DSDL compiler.
         * This check shall never be performed in user code because MaxBitLen value
         * actually depends on the nested types, thus it is not invariant.
         */
        ::uavcan::StaticAssert<56 == MaxBitLen>::check();
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
    enum { DefaultDataTypeID = 4 };

    static const char* getDataTypeFullName()
    {
        return "uavcan.protocol.GlobalTimeSync";
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
bool GlobalTimeSync_<_tmpl>::operator==(ParameterType rhs) const
{
    return
        previous_transmission_timestamp_usec == rhs.previous_transmission_timestamp_usec;
}

template <int _tmpl>
bool GlobalTimeSync_<_tmpl>::isClose(ParameterType rhs) const
{
    return
        ::uavcan::areClose(previous_transmission_timestamp_usec, rhs.previous_transmission_timestamp_usec);
}

template <int _tmpl>
int GlobalTimeSync_<_tmpl>::encode(ParameterType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::previous_transmission_timestamp_usec::encode(self.previous_transmission_timestamp_usec, codec,  tao_mode);
    return res;
}

template <int _tmpl>
int GlobalTimeSync_<_tmpl>::decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::previous_transmission_timestamp_usec::decode(self.previous_transmission_timestamp_usec, codec,  tao_mode);
    return res;
}

/*
 * Out of line type method definitions
 */
template <int _tmpl>
::uavcan::DataTypeSignature GlobalTimeSync_<_tmpl>::getDataTypeSignature()
{
    ::uavcan::DataTypeSignature signature(0x20271116A793C2DBULL);

    FieldTypes::previous_transmission_timestamp_usec::extendDataTypeSignature(signature);

    return signature;
}

/*
 * Out of line constant definitions
 */

template <int _tmpl>
const typename ::uavcan::StorageType< typename GlobalTimeSync_<_tmpl>::ConstantTypes::MAX_BROADCASTING_PERIOD_MS >::Type
    GlobalTimeSync_<_tmpl>::MAX_BROADCASTING_PERIOD_MS = 1100U; // 1100

template <int _tmpl>
const typename ::uavcan::StorageType< typename GlobalTimeSync_<_tmpl>::ConstantTypes::MIN_BROADCASTING_PERIOD_MS >::Type
    GlobalTimeSync_<_tmpl>::MIN_BROADCASTING_PERIOD_MS = 40U; // 40

template <int _tmpl>
const typename ::uavcan::StorageType< typename GlobalTimeSync_<_tmpl>::ConstantTypes::RECOMMENDED_BROADCASTER_TIMEOUT_MS >::Type
    GlobalTimeSync_<_tmpl>::RECOMMENDED_BROADCASTER_TIMEOUT_MS = 2200U; // 2200

/*
 * Final typedef
 */
typedef GlobalTimeSync_<0> GlobalTimeSync;

namespace
{

const ::uavcan::DefaultDataTypeRegistrator< ::uavcan::protocol::GlobalTimeSync > _uavcan_gdtr_registrator_GlobalTimeSync;

}

} // Namespace protocol
} // Namespace uavcan

/*
 * YAML streamer specialization
 */
namespace uavcan
{

template <>
class UAVCAN_EXPORT YamlStreamer< ::uavcan::protocol::GlobalTimeSync >
{
public:
    template <typename Stream>
    static void stream(Stream& s, ::uavcan::protocol::GlobalTimeSync::ParameterType obj, const int level);
};

template <typename Stream>
void YamlStreamer< ::uavcan::protocol::GlobalTimeSync >::stream(Stream& s, ::uavcan::protocol::GlobalTimeSync::ParameterType obj, const int level)
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
    s << "previous_transmission_timestamp_usec: ";
    YamlStreamer< ::uavcan::protocol::GlobalTimeSync::FieldTypes::previous_transmission_timestamp_usec >::stream(s, obj.previous_transmission_timestamp_usec, level + 1);
}

}

namespace uavcan
{
namespace protocol
{

template <typename Stream>
inline Stream& operator<<(Stream& s, ::uavcan::protocol::GlobalTimeSync::ParameterType obj)
{
    ::uavcan::YamlStreamer< ::uavcan::protocol::GlobalTimeSync >::stream(s, obj, 0);
    return s;
}

} // Namespace protocol
} // Namespace uavcan

#endif // UAVCAN_PROTOCOL_GLOBALTIMESYNC_HPP_INCLUDED