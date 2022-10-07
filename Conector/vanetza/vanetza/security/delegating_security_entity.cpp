#include <vanetza/common/its_aid.hpp>
#include <vanetza/security/delegating_security_entity.hpp>
#include <stdexcept>
#include "log.hpp"

using namespace log_module;

namespace vanetza
{
    namespace security
    {

        DelegatingSecurityEntity::DelegatingSecurityEntity(SignService sign, VerifyService verify) : m_sign_service(std::move(sign)),
                                                                                                     m_verify_service(std::move(verify))
        {
            if (!m_sign_service)
            {
                throw std::invalid_argument("SN-SIGN service is not callable");
            }
            else if (!m_verify_service)
            {
                throw std::invalid_argument("SN-VERIFY service is not callable");
            }
        }

        EncapConfirm DelegatingSecurityEntity::encapsulate_packet(EncapRequest &&encap_request)
        {
            SignRequest sign_request;
            sign_request.plain_message = std::move(encap_request.plaintext_payload);
            sign_request.its_aid = encap_request.its_aid;

            SignConfirm sign_confirm = m_sign_service(std::move(sign_request));
            EncapConfirm encap_confirm;
            encap_confirm.sec_packet = std::move(sign_confirm.secured_message);
            return encap_confirm;
        }

        void print_confirm(vanetza::security::VerificationReport r)
        {
            printf("Confirm report: ");
            switch (r)
            {
            case VerificationReport::Success:
                printf("VerificationReport::Success\n");
                break;

            case VerificationReport::Success_Certificate:
                printf("VerificationReport::Success_Certificate\n");
                break;

            case VerificationReport::Signer_Certificate_Not_Found_v3:
                printf("VerificationReport::Signer_Certificate_Not_Found_v3\n");
                break;

            case VerificationReport::Invalid_Certificate:
                printf("VerificationReport::Invalid_Certificate\n");
                break;

            case VerificationReport::Invalid_Timestamp:
                printf("VerificationReport::Invalid_Timestamp\n");
                break;

            }
        }

        void print_decap(vanetza::security::DecapReport r)
        {
            printf("Confirm report: ");
            switch (r)
            {
            case DecapReport::Success:
                printf("DecapReport::Success\n");
                break;

            case DecapReport::Success_Certificate:
                printf("DecapReport::Success_Certificate\n");
                break;

            case DecapReport::Signer_Certificate_Not_Found_v3:
                printf("DecapReportReport::Signer_Certificate_Not_Found_v3\n");
                break;

            case DecapReport::Invalid_Certificate:
                printf("DecapReportReport::Invalid_Certificate\n");
                break;
            }
        }

        DecapConfirm DelegatingSecurityEntity::decapsulate_packet(DecapRequest &&decap_request)
        {
            print_log("**DelegatingSecurityEntity::decapsulate_packet**");
            VerifyConfirm verify_confirm = m_verify_service(VerifyRequest{decap_request.sec_packet});
            DecapConfirm decap_confirm;
            decap_confirm.plaintext_payload = std::move(decap_request.sec_packet.payload.data);

            print_confirm(verify_confirm.report);

            decap_confirm.report = static_cast<DecapReport>(verify_confirm.report);

            print_decap(decap_confirm.report);

            decap_confirm.certificate_validity = verify_confirm.certificate_validity;
            decap_confirm.its_aid = verify_confirm.its_aid;
            decap_confirm.permissions = verify_confirm.permissions;
            return decap_confirm;
        }

    } // namespace security
} // namespace vanetza
