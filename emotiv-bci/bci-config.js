// Get your client id and client secret from https://www.emotiv.com/my-account/cortex-apps/
// Use your client id and client secret with cortex example and Cortex UI to accept EULA before using it with node-red

module.exports = {
  path        : 'wss://127.0.0.1:6868/com.emotiv.noderedtoolbox',
  clientid    : 's5zWRoJY3vHPftnltiXw5f8vmnJPswWarVtYgbsc',
  clientsecret: 'uKpcDqqk179w17rV8RsSfmxfG2FCAe5bA6NT6RTcAdffOByU7kjhHntcSdeypfCbqE9cnicZpqYMGR9ask8HTylA0Kjxk2l4lzNP6MtwRmIjdjh5pmo8ngdlHoYWHPvv',
  errorCode: {
    ERROR_USER_LOGGED_IN      : -32032,
    ERR_NO_HEADSET            : -32004,
    ERR_INVALID_PROFILE       : -32031,
    ERR_INVALID_USERNAME_PWD  : -32001,
    ERR_REQUEST_TIME_OUT      : -32039,
    ERR_PROCESSING_PROFILE    : -32036,
    ERR_SESSION_DOES_NOT_EXIST: -32007,
    ERR_SESSION_CONFLICT      : -32005,
    ERR_EULA_IS_NOT_ACCEPTED  : -32041,
    ERR_PROFILE_CONFLICT      : -32044,
    ERR_PROFILE_LOADED_BY_ANOTHER_APP: -32046
  },
  maxSocketListener: 0,
  stream           : ['com']
};
