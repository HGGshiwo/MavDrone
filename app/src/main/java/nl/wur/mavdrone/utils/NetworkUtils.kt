package nl.wur.mavdrone.utils

import android.content.Context
import java.net.Inet4Address
import java.net.NetworkInterface

class NetworkUtils {
    companion object {
        private val LOG = Logger("NetworkUtils")

        // 地面站经 WiFi/以太网直连本机, 而系统 activeNetwork 可能落在蜂窝(10.x)、
        // USB 共享或 VPN(172.16-31.x)网卡上, 因此必须遍历网卡按可达性选地址,
        // 而不是取默认网络的第一个 IPv4
        @Suppress("UNUSED_PARAMETER")
        fun getLocalWifiIpAddress(context: Context): String? {
            val preferred = ArrayList<String>()
            val siteLocal = ArrayList<String>()
            val candidates = ArrayList<String>()

            for (nif in NetworkInterface.getNetworkInterfaces()) {
                if (!nif.isUp || nif.isLoopback) continue
                for (address in nif.inetAddresses) {
                    if (address.isLoopbackAddress || address !is Inet4Address) continue
                    val ip = address.hostAddress ?: continue
                    val name = nif.name.lowercase()
                    candidates.add("$name=$ip")

                    if (name.startsWith("wlan") || name.startsWith("eth") || name.startsWith("ap") || name.startsWith("swlan")) {
                        preferred.add(ip)
                    } else if (address.isSiteLocalAddress) {
                        siteLocal.add(ip)
                    }
                }
            }

            LOG.i("Network interfaces: %s", candidates.joinToString(", "))

            val result = preferred.firstOrNull() ?: siteLocal.firstOrNull() ?: candidates.firstOrNull()?.substringAfterLast("=")
            LOG.i("Selected server IP: %s", result)
            return result
        }
    }
}
