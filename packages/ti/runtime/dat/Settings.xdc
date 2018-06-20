
module Settings
{
    /*! This definition controls the device type selection
     * for the given library.
     *
     * To use the library for a specific device, add the following lines to the
     * applications RTSC configuration file:
     *
     *      var Dat = xdc.useModule ('ti.runtime.dat.Settings');
     *      Dat.deviceType = "device_name"
     *
     * Supported device_name
     *      k2h -> Hawking
     *      k2l -> Lamarr
     *
     * By default this variable is set to NULL. Applications need to explicitly
     * select the device.
     */
    metaonly config string deviceType = "";
}


