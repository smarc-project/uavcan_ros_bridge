#
# Abstract node status information.
#
# Any UAVCAN node is required to publish this message periodically.
#

#
# Publication period may vary within these limits.
# It is NOT recommended to change it at run time.
#
uint16 MAX_BROADCASTING_PERIOD_MS = 1000
uint16 MIN_BROADCASTING_PERIOD_MS = 2

#
# If a node fails to publish this message in this amount of time, it should be considered offline.
#
uint16 OFFLINE_TIMEOUT_MS = 3000

#
# Uptime counter should never overflow.
# Other nodes may detect that a remote node has restarted when this value goes backwards.
#
uint32 uptime_sec

#
# Abstract node health.
#
uint8 HEALTH_OK         = 0     # The node is functioning properly.
uint8 HEALTH_WARNING    = 1     # A critical parameter went out of range or the node encountered a minor failure.
uint8 HEALTH_ERROR      = 2     # The node encountered a major failure.
uint8 HEALTH_CRITICAL   = 3     # The node suffered a fatal malfunction.
uint8 health

#
# Current mode.
#
# Mode OFFLINE can be actually reported by the node to explicitly inform other network
# participants that the sending node is about to shutdown. In this case other nodes will not
# have to wait OFFLINE_TIMEOUT_MS before they detect that the node is no longer available.
#
# Reserved values can be used in future revisions of the specification.
#
uint8 MODE_OPERATIONAL      = 0         # Node is performing its main functions.
uint8 MODE_INITIALIZATION   = 1         # Node is initializing; this mode is entered immediately after startup.
uint8 MODE_MAINTENANCE      = 2         # Node is under maintenance.
uint8 MODE_SOFTWARE_UPDATE  = 3         # Node is in the process of updating its software.
uint8 MODE_OFFLINE          = 7         # Node is no longer available.
uint8 mode

#
# Not used currently, keep zero when publishing, ignore when receiving.
#
uint8 sub_mode

#
# Optional, vendor-specific node status code, e.g. a fault code or a status bitmask.
#
uint16 vendor_specific_status_code
