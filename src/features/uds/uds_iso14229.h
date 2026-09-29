#ifndef UDS_ISO14229_H
#define UDS_ISO14229_H

/*
 * TEMPORARY: generic ISO 14229-1 protocol constants that gen/vehicle_cl250.h does not
 * provide yet (user decision 2026-09-29). Only service and response codes defined by
 * the standard live here, never vehicle facts: IDs, DIDs, formulas, timings and the
 * session/tester-present requests come from gen/. Whether a request may be sent is
 * decided by the generated D-020 gates and the can_if guard, not by this header.
 * Remove it once moto-vehicle-defs generates these codes (see features/uds/README.md).
 */

/* ReadDataByIdentifier request SID (ISO 14229-1 §11.2); allowed by tester_policy. */
#define UDS_SID_READ_DATA_BY_IDENTIFIER 0x22u

/* Negative response: [0x7F][request SID][NRC] (ISO 14229-1 §8.7.2). */
#define UDS_SID_NEGATIVE_RESPONSE 0x7Fu
#define UDS_NEGATIVE_RESPONSE_LEN 3u

/* NRCs (ISO 14229-1 Annex A.1) the client acts on. */
#define UDS_NRC_RESPONSE_PENDING 0x78u
#define UDS_NRC_SUBFUNCTION_NOT_SUPPORTED_IN_ACTIVE_SESSION 0x7Eu
#define UDS_NRC_SERVICE_NOT_SUPPORTED_IN_ACTIVE_SESSION 0x7Fu

#endif /* UDS_ISO14229_H */
