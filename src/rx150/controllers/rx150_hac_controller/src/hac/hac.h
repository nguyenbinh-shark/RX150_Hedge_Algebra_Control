#ifndef RX150_HAC_CONTROLLER_HAC_H
#define RX150_HAC_CONTROLLER_HAC_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Evaluate the stateless two-input HAC law. Inputs are normalized and
 * clamped by the caller; the caller also clamps the returned control value.
 */
float hac_eval(float x, float xd, float a, float b, float c);

#ifdef __cplusplus
}
#endif

#endif  /* RX150_HAC_CONTROLLER_HAC_H */
