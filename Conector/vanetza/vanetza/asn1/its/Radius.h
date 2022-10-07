/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "CPM-PDU-Descriptions"
 * 	found in "asn1/TR103562v211.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example -R`
 */

#ifndef	_Radius_H_
#define	_Radius_H_


#include "asn_application.h"

/* Including external dependencies */
#include "NativeInteger.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum Radius {
	Radius_zeroPointOneMeter	= 1,
	Radius_oneMeter	= 10
} e_Radius;

/* Radius */
typedef long	 Radius_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_Radius_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_Radius;
asn_struct_free_f Radius_free;
asn_struct_print_f Radius_print;
asn_constr_check_f Radius_constraint;
ber_type_decoder_f Radius_decode_ber;
der_type_encoder_f Radius_encode_der;
xer_type_decoder_f Radius_decode_xer;
xer_type_encoder_f Radius_encode_xer;
oer_type_decoder_f Radius_decode_oer;
oer_type_encoder_f Radius_encode_oer;
per_type_decoder_f Radius_decode_uper;
per_type_encoder_f Radius_encode_uper;
per_type_decoder_f Radius_decode_aper;
per_type_encoder_f Radius_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _Radius_H_ */
#include "asn_internal.h"
