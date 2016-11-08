/*
 * Generated by asn1c-0.9.27 (http://lionet.info/asn1c)
 * From ASN.1 module "ITS-Container"
 * 	found in "/home/rieblr/work/car2x/vanetza/vanetza/asn1/cdd_ts102894-2v1.2.1.asn1"
 * 	`asn1c -fcompound-names -gen-PER`
 */

#ifndef	_CollisionRiskSubCauseCode_H_
#define	_CollisionRiskSubCauseCode_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeInteger.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum CollisionRiskSubCauseCode {
	CollisionRiskSubCauseCode_unavailable	= 0,
	CollisionRiskSubCauseCode_longitudinalCollisionRisk	= 1,
	CollisionRiskSubCauseCode_crossingCollisionRisk	= 2,
	CollisionRiskSubCauseCode_lateralCollisionRisk	= 3,
	CollisionRiskSubCauseCode_vulnerableRoadUser	= 4
} e_CollisionRiskSubCauseCode;

/* CollisionRiskSubCauseCode */
typedef long	 CollisionRiskSubCauseCode_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_CollisionRiskSubCauseCode;
asn_struct_free_f CollisionRiskSubCauseCode_free;
asn_struct_print_f CollisionRiskSubCauseCode_print;
asn_constr_check_f CollisionRiskSubCauseCode_constraint;
ber_type_decoder_f CollisionRiskSubCauseCode_decode_ber;
der_type_encoder_f CollisionRiskSubCauseCode_encode_der;
xer_type_decoder_f CollisionRiskSubCauseCode_decode_xer;
xer_type_encoder_f CollisionRiskSubCauseCode_encode_xer;
per_type_decoder_f CollisionRiskSubCauseCode_decode_uper;
per_type_encoder_f CollisionRiskSubCauseCode_encode_uper;

#ifdef __cplusplus
}
#endif

#endif	/* _CollisionRiskSubCauseCode_H_ */
#include <asn_internal.h>
