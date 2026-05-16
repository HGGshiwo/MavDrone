package nl.wur.mavdrone.utils

import android.content.Context
import android.net.ConnectivityManager

class NetworkUtils {
    companion object {
                fun getLocalWifiIpAddress(context: Context): String? {
            val connectivityManager = context.getSystemService(Context.CONNECTIVITY_SERVICE) as ConnectivityManager
            val linkAddresses = connectivityManager.getLinkProperties(connectivityManager.activeNetwork)?.linkAddresses

            val ipV4Address = linkAddresses?.firstOrNull { linkAddress ->
                linkAddress.address.hostAddress?.contains('.') ?: false }?.address?.hostAddress

            return ipV4Address
        }
    }
}
