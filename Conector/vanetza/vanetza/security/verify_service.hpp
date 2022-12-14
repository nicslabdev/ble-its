#ifndef VERIFY_SERVICE_HPP_BR4ISDBH
#define VERIFY_SERVICE_HPP_BR4ISDBH

#include <boost/optional.hpp>
#include <vanetza/common/its_aid.hpp>
#include <vanetza/common/byte_buffer.hpp>
#include <vanetza/security/certificate.hpp>
#include <vanetza/security/int_x.hpp>
#include <vanetza/security/secured_message.hpp>
#include <vanetza/security/cam_store.hpp>
#include <functional>

namespace vanetza
{

// forward declaration
class PositionProvider;
class Runtime;

namespace security
{

// forward declarations
class Backend;
class CertificateCache;
class CertificateProvider;
class CertificateValidator;
class SignHeaderPolicy;

enum class VerificationReport
{
    Success,
    Success_Certificate,
    False_Signature,
    Invalid_Certificate,
    Revoked_Certificate,
    Inconsistent_Chain,
    Invalid_Timestamp,
    Duplicate_Message,
    Invalid_Mobility_Data,
    Unsigned_Message,
    Signer_Certificate_Not_Found,
    Signer_Certificate_Not_Found_v2, //mod verify service
    Signer_Certificate_Not_Found_v3, //mod verify service
    Unsupported_Signer_Identifier_Type,
    Incompatible_Protocol
};


// mandatory parameters of SN-VERIFY.request (TS 102 723-8 V1.1.1)
struct VerifyRequest
{
    VerifyRequest(const SecuredMessage& msg) : secured_message(msg) {}
    const SecuredMessage& secured_message; /*< contains security header and payload */
};

// parameters of SN-VERIFY.confirm (TS 102 723-8 V1.1.1)
struct VerifyConfirm
{
    VerificationReport report; // mandatory
    ItsAid its_aid; // mandatory
    ByteBuffer permissions; // mandatory
    CertificateValidity certificate_validity; // non-standard extension
    boost::optional<HashedId8> certificate_id; // optional
};

/**
 * Equivalent of SN-VERIFY service in TS 102 723-8 V1.1.1
 */
using VerifyService = std::function<VerifyConfirm(VerifyRequest&&)>;

/**
 * Get verify service with basic certificate and signature checks
 * \param rt runtime
 * \param certificate_provider certificate provider
 * \param certificate_validator certificate validator
 * \param backend crypto backend
 * \param certificate_cache certificate cache
 * \param sign_header_policy sign header policy to report unknown certificates
 * \return callable verify service
 */
VerifyService straight_verify_service(const Runtime&, CertificateProvider&, CertificateValidator&, Backend&, CertificateCache&, SignHeaderPolicy&, PositionProvider&);



/** MODIFICATION
 * Get verify service with basic certificate and signature checks
 * \param rt runtime
 * \param certificate_provider certificate provider
 * \param certificate_validator certificate validator
 * \param backend crypto backend
 * \param certificate_cache certificate cache
 * \param sign_header_policy sign header policy to report unknown certificates
 * \param cert_cache_v3 certificate cache
 * \param frames stored frames waiting for verification
 * \param soft_ver true means that soft verification is enabled
 * \return callable verify service
 */
VerifyService straight_verify_service_mod(const Runtime& rt, CertificateProvider& cert_provider, CertificateValidator& certs, Backend& backend, CertificateCache& cert_cache_v3, CamStore& frames, SignHeaderPolicy& sign_policy, PositionProvider& positioning, bool soft_ver);

/**
 * Get insecure dummy verify service without any checks
 * \param report confirm report result
 * \param validity certificate validity result
 * \return callable verify service
 */
VerifyService dummy_verify_service(VerificationReport report, CertificateValidity validity);

} // namespace security
} // namespace vanetza

#endif /* VERIFY_SERVICE_HPP_BR4ISDBH */
