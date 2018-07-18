/*
 * UAVCAN data structure definition for libuavcan.
 *
 * Autogenerated, do not edit.
 *
 * Source file: /data/temp/PA-FlightControl/src/modules/uavcan/libuavcan/libuavcan/test/dsdl_test/root_ns_a/B.uavcan
 */

#ifndef ROOT_NS_A_B_HPP_INCLUDED
#define ROOT_NS_A_B_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/node/global_data_type_registry.hpp>
#include <uavcan/marshal/types.hpp>

/******************************* Source text **********************************
float64[2] vector
bool[16] bools
******************************************************************************/

/********************* DSDL signature source definition ***********************
root_ns_a.B
saturated float64[2] vector
saturated bool[16] bools
******************************************************************************/

#undef vector
#undef bools

namespace root_ns_a
{

template <int _tmpl>
struct UAVCAN_EXPORT B_
{
    typedef const B_<_tmpl>& ParameterType;
    typedef B_<_tmpl>& ReferenceType;

    struct ConstantTypes
    {
    };

    struct FieldTypes
    {
        typedef ::uavcan::Array< ::uavcan::FloatSpec< 64, ::uavcan::CastModeSaturate >, ::uavcan::ArrayModeStatic, 2 > vector;
        typedef ::uavcan::Array< ::uavcan::IntegerSpec< 1, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate >, ::uavcan::ArrayModeStatic, 16 > bools;
    };

    enum
    {
        MinBitLen
            = FieldTypes::vector::MinBitLen
            + FieldTypes::bools::MinBitLen
    };

    enum
    {
        MaxBitLen
            = FieldTypes::vector::MaxBitLen
            + FieldTypes::bools::MaxBitLen
    };

    // Constants

    // Fields
    typename ::uavcan::StorageType< typename FieldTypes::vector >::Type vector;
    typename ::uavcan::StorageType< typename FieldTypes::bools >::Type bools;

    B_()
        : vector()
        , bools()
    {
        ::uavcan::StaticAssert<_tmpl == 0>::check();  // Usage check

#if UAVCAN_DEBUG
        /*
         * Cross-checking MaxBitLen provided by the DSDL compiler.
         * This check shall never be performed in user code because MaxBitLen value
         * actually depends on the nested types, thus it is not invariant.
         */
        ::uavcan::StaticAssert<144 == MaxBitLen>::check();
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
    // This type has no default data type ID

    static const char* getDataTypeFullName()
    {
        return "root_ns_a.B";
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
bool B_<_tmpl>::operator==(ParameterType rhs) const
{
    return
        vector == rhs.vector &&
        bools == rhs.bools;
}

template <int _tmpl>
bool B_<_tmpl>::isClose(ParameterType rhs) const
{
    return
        ::uavcan::areClose(vector, rhs.vector) &&
        ::uavcan::areClose(bools, rhs.bools);
}

template <int _tmpl>
int B_<_tmpl>::encode(ParameterType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::vector::encode(self.vector, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::bools::encode(self.bools, codec,  tao_mode);
    return res;
}

template <int _tmpl>
int B_<_tmpl>::decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::vector::decode(self.vector, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::bools::decode(self.bools, codec,  tao_mode);
    return res;
}

/*
 * Out of line type method definitions
 */
template <int _tmpl>
::uavcan::DataTypeSignature B_<_tmpl>::getDataTypeSignature()
{
    ::uavcan::DataTypeSignature signature(0xCE758EF6221573DULL);

    FieldTypes::vector::extendDataTypeSignature(signature);
    FieldTypes::bools::extendDataTypeSignature(signature);

    return signature;
}

/*
 * Out of line constant definitions
 */

/*
 * Final typedef
 */
typedef B_<0> B;

// No default registration

} // Namespace root_ns_a

/*
 * YAML streamer specialization
 */
namespace uavcan
{

template <>
class UAVCAN_EXPORT YamlStreamer< ::root_ns_a::B >
{
public:
    template <typename Stream>
    static void stream(Stream& s, ::root_ns_a::B::ParameterType obj, const int level);
};

template <typename Stream>
void YamlStreamer< ::root_ns_a::B >::stream(Stream& s, ::root_ns_a::B::ParameterType obj, const int level)
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
    s << "vector: ";
    YamlStreamer< ::root_ns_a::B::FieldTypes::vector >::stream(s, obj.vector, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "bools: ";
    YamlStreamer< ::root_ns_a::B::FieldTypes::bools >::stream(s, obj.bools, level + 1);
}

}

namespace root_ns_a
{

template <typename Stream>
inline Stream& operator<<(Stream& s, ::root_ns_a::B::ParameterType obj)
{
    ::uavcan::YamlStreamer< ::root_ns_a::B >::stream(s, obj, 0);
    return s;
}

} // Namespace root_ns_a

#endif // ROOT_NS_A_B_HPP_INCLUDED