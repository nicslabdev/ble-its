/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "CPM-PDU-Descriptions"
 * 	found in "asn1/TR103562v211.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example -R`
 */

#ifndef	_VehicleSubclassType_H_
#define	_VehicleSubclassType_H_


#include "asn_application.h"

/* Including external dependencies */
#include "NativeInteger.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum VehicleSubclassType {
	VehicleSubclassType_unknown	= 0,
	VehicleSubclassType_moped	= 1,
	VehicleSubclassType_motorcycle	= 2,
	VehicleSubclassType_passengerCar	= 3,
	VehicleSubclassType_bus	= 4,
	VehicleSubclassType_lightTruck	= 5,
	VehicleSubclassType_heavyTruck	= 6,
	VehicleSubclassType_trailer	= 7,
	VehicleSubclassType_specialVehicles	= 8,
	VehicleSubclassType_tram	= 9,
	VehicleSubclassType_emergencyVehicle	= 10,
	VehicleSubclassType_agricultural	= 11
} e_VehicleSubclassType;

/* VehicleSubclassType */
typedef long	 VehicleSubclassType_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_VehicleSubclassType_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_VehicleSubclassType;
asn_struct_free_f VehicleSubclassType_free;
asn_struct_print_f VehicleSubclassType_print;
asn_constr_check_f VehicleSubclassType_constraint;
ber_type_decoder_f VehicleSubclassType_decode_ber;
der_type_encoder_f VehicleSubclassType_encode_der;
xer_type_decoder_f VehicleSubclassType_decode_xer;
xer_type_encoder_f VehicleSubclassType_encode_xer;
oer_type_decoder_f VehicleSubclassType_decode_oer;
oer_type_encoder_f VehicleSubclassType_encode_oer;
per_type_decoder_f VehicleSubclassType_decode_uper;
per_type_encoder_f VehicleSubclassType_encode_uper;
per_type_decoder_f VehicleSubclassType_decode_aper;
per_type_encoder_f VehicleSubclassType_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _VehicleSubclassType_H_ */
#include "asn_internal.h"
