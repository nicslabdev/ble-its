#ifndef DENM_HPP_XGC8NRDI
#define DENM_HPP_XGC8NRDI

#include <vanetza/asn1/asn1c_conversion.hpp>
#include <vanetza/asn1/asn1c_wrapper.hpp>
#include <vanetza/asn1/its/DENM.h>

namespace vanetza
{
namespace asn1
{

class Denm : public asn1c_per_wrapper<DENM_t>
{
public:
    using wrapper = asn1c_per_wrapper<DENM_t>;
    Denm() : wrapper(asn_DEF_DENM) {}
};

} // namespace asn1
} // namespace vanetza

#endif /* DENM_HPP_XGC8NRDI */

