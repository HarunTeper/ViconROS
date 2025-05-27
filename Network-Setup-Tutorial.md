# ROS 2 Cyclone DDS Networking Tutorial: Windows (WSL2+Docker) ↔ Linux over WiFi

This guide walks you through making **ROS 2 nodes running in Docker on WSL2** fully communicate with ROS 2 nodes on a **Linux machine** over a **WiFi network**, using **Cyclone DDS**.

---

## 📉 Prerequisites

* Windows 11 with **WSL2 installed**
* Ubuntu in WSL2 with **Docker installed and using host networking**
* A **Linux machine** (e.g., Ubuntu 22.04) on the **same WiFi network**
* **ROS 2 Humble** or compatible installed
* Basic ROS 2 and Docker knowledge

---

## 🛠️ Step-by-Step Instructions

### ✅ Step 1: Ensure WSL2 is Using **Mirrored Networking**

To allow WSL2 to use the same IP as the host (WiFi interface), ensure that **mirrored networking** is enabled. On Windows 11 23H2 and later, this is the default.

You can verify by checking the IP inside WSL matches your host’s WiFi IP:

```bash
ip addr show
```

Look for a device with an IP like `192.168.X.Y`. This should match the host's WiFi IP.

---

### 🔐 Step 2: Create Windows Firewall Rules

Create inbound and outbound rules for **UDP ports 7400–7500**, used by **Cyclone DDS** for discovery and communication.

#### 📋 PowerShell Commands:

```powershell
# Inbound
New-NetFirewallRule -DisplayName "CycloneDDS-Inbound" -Direction Inbound -Protocol UDP -LocalPort 7400-7500 -Action Allow

# Outbound
New-NetFirewallRule -DisplayName "CycloneDDS-Outbound" -Direction Outbound -Protocol UDP -LocalPort 7400-7500 -Action Allow
```

---

### 🔥 Step 3: Enable Hyper-V VM Inbound Traffic

If you're running Docker inside WSL2, traffic can be blocked by Hyper-V virtual switch isolation. Allow inbound traffic for your WSL instance using:

```powershell
Set-NetFirewallHyperVVMSetting -Name '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' -DefaultInboundAction Allow
```

---

### ⚙️ Step 4: Cyclone DDS Configuration

Create a `cyclonedds.xml` file on **each machine**, and mount it into the Docker container.

#### 📄 Example config (`cyclonedds.xml`):

```xml
<CycloneDDS>
   <Domain>
      <General>
         <Interfaces>
            <NetworkInterface name="YOUR_WIFI_INTERFACE_NAME"/>
         </Interfaces>
      </General>
   </Domain>
</CycloneDDS>
```

* Replace `YOUR_WIFI_INTERFACE_NAME` with your WiFi interface name:

  * On Linux: `wlp3s0`, `wlan0`, etc.
  * On Windows/WSL2: `eth0`, `Wi-Fi`, or check with `ip addr show`.

✨ You can also use `address="192.168.X.Y"` instead of `name` if preferred.

---

### ⚒️ Step 5: Test UDP Connectivity with `nc`

Before running ROS 2, test UDP communication using netcat:

#### On Machine A (WSL2/Docker):

```bash
nc -ul 7450
```

#### On Machine B (Linux):

```bash
echo "Hello from Linux" | nc -u 192.168.X.Y 7450
```

If the message appears on Machine A, UDP works correctly.

Reverse the roles to test both directions.

---

### 🚀 Step 6: Run ROS 2 Nodes

Now, launch ROS 2 containers on both sides using `--network=host` and mount the `cyclonedds.xml` config:

On one side, run:

```bash
ros2 topic list
```

On the other, run:

```bash
ros2 topic pub /chatter std_msgs/String '{data: "Hello from ROS 2"}'
```

Check if topics from both machines are visible. Check both directions.

---

## ✅ Summary

| Component         | Required? | Purpose                               |
| ----------------- | --------- | ------------------------------------- |
| Mirrored WSL2     | ✅         | Shares WiFi IP between host and WSL2  |
| Firewall Rules    | ✅         | Allows DDS traffic on 7400–7500 (UDP) |
| Hyper-V Allow     | ✅         | Enables WSL2 inbound UDP              |
| CycloneDDS XML    | ✅         | Sets correct interface for DDS comms  |
| UDP Test via `nc` | ✅         | Verifies networking setup             |
