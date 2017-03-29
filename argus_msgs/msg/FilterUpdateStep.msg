# Message detailing a filter update step
#
# Fields
# ======
# prior_state_cov : State covariance before update
# prior_obs_error : Observation prediction error before update
# obs_error_cov   : Theoretical covariance of prediction error before update
# post_state_cov  : State covariance after update
# state_delta     : Correction applied to state from update
# post_obs_error  : Observation prediction error after update
# obs_jacobian    : Observation function jacobian
# obs_noise_cov   : Observation noise covariance 

MatrixFloat64 prior_state_cov
float64[] prior_obs_error
MatrixFloat64 obs_error_cov

MatrixFloat64 post_state_cov
float64[] state_delta
float64[] post_obs_error

MatrixFloat64 obs_jacobian
MatrixFloat64 obs_noise_cov