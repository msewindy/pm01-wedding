#!/bin/bash
# 网络和防火墙设置重置脚本
# 将系统网络和防火墙配置恢复为默认值

set -e

echo "=========================================="
echo "开始重置网络和防火墙设置到默认值"
echo "=========================================="

# 1. 重置防火墙设置
echo ""
echo "1. 重置防火墙设置..."

# 检查并重置UFW
if command -v ufw &> /dev/null; then
    echo "   - 重置UFW防火墙..."
    sudo ufw --force reset
    sudo ufw default deny incoming
    sudo ufw default allow outgoing
    sudo ufw --force disable
    echo "   ✓ UFW已重置并禁用"
fi

# 检查并重置firewalld
if command -v firewall-cmd &> /dev/null; then
    echo "   - 重置firewalld防火墙..."
    if sudo systemctl is-active --quiet firewalld; then
        sudo firewall-cmd --reload
        echo "   ✓ firewalld已重新加载"
    else
        echo "   - firewalld未运行，跳过"
    fi
fi

# 重置iptables到默认策略
echo "   - 重置iptables规则..."
sudo iptables -P INPUT ACCEPT
sudo iptables -P FORWARD ACCEPT
sudo iptables -P OUTPUT ACCEPT
sudo iptables -F
sudo iptables -X
sudo iptables -t nat -F
sudo iptables -t nat -X
sudo iptables -t mangle -F
sudo iptables -t mangle -X
echo "   ✓ iptables已重置"

# 2. 重置网络内核参数到默认值
echo ""
echo "2. 重置网络内核参数..."

# 重置IP转发（默认关闭）
sudo sysctl -w net.ipv4.ip_forward=0
sudo sysctl -w net.ipv6.conf.all.forwarding=0

# 重置其他常见的网络安全参数到默认值
sudo sysctl -w net.ipv4.conf.all.accept_redirects=0
sudo sysctl -w net.ipv4.conf.all.accept_source_route=0
sudo sysctl -w net.ipv4.conf.all.send_redirects=0
sudo sysctl -w net.ipv4.conf.all.rp_filter=1
sudo sysctl -w net.ipv4.conf.default.accept_redirects=0
sudo sysctl -w net.ipv4.conf.default.accept_source_route=0
sudo sysctl -w net.ipv4.conf.default.send_redirects=0
sudo sysctl -w net.ipv4.conf.default.rp_filter=1

# 使sysctl设置永久生效
if [ -f /etc/sysctl.conf ]; then
    echo "   - 备份现有sysctl.conf..."
    sudo cp /etc/sysctl.conf /etc/sysctl.conf.backup.$(date +%Y%m%d_%H%M%S)
fi

# 移除自定义网络参数（保留系统默认）
sudo sed -i '/^net\./d' /etc/sysctl.conf 2>/dev/null || true

echo "   ✓ 网络内核参数已重置"

# 3. 重置DNS设置（使用系统默认）
echo ""
echo "3. 检查DNS设置..."
# DNS由systemd-resolved管理，通常不需要手动重置
# 如果需要重置，可以重启systemd-resolved服务
if systemctl is-active --quiet systemd-resolved; then
    echo "   - systemd-resolved正在运行（正常）"
    echo "   - 当前DNS服务器: $(resolvectl status | grep 'DNS Server' | head -1 | awk '{print $3}')"
else
    echo "   - systemd-resolved未运行"
fi

# 4. 重置NetworkManager连接设置（可选）
echo ""
echo "4. 检查NetworkManager配置..."
if command -v nmcli &> /dev/null; then
    echo "   - NetworkManager正在运行"
    echo "   - 当前连接:"
    nmcli connection show --active | grep -v "^NAME" | awk '{print "     " $0}'
    echo ""
    echo "   注意: NetworkManager连接配置将保持原样"
    echo "   如需重置特定连接，请手动操作"
else
    echo "   - NetworkManager未安装"
fi

# 5. 检查netplan配置
echo ""
echo "5. 检查netplan配置..."
if [ -f /etc/netplan/01-network-manager-all.yaml ]; then
    echo "   - netplan配置文件存在"
    echo "   - 当前配置:"
    cat /etc/netplan/01-network-manager-all.yaml | sed 's/^/     /'
    echo ""
    echo "   注意: netplan配置已经是默认的NetworkManager模式"
fi

# 6. 显示当前网络状态
echo ""
echo "6. 当前网络状态:"
echo "   - 活动网络接口:"
ip addr show | grep -E "^[0-9]+:|inet " | grep -v "127.0.0.1" | sed 's/^/     /'
echo ""
echo "   - 路由表:"
ip route show | sed 's/^/     /'

echo ""
echo "=========================================="
echo "重置完成！"
echo "=========================================="
echo ""
echo "注意事项:"
echo "1. 防火墙已重置为默认状态（通常为关闭或仅允许出站）"
echo "2. IP转发已关闭（默认值）"
echo "3. 网络接口配置由NetworkManager管理，保持不变"
echo "4. DNS配置由systemd-resolved管理，使用DHCP获取的DNS服务器"
echo ""
echo "如需重新启用防火墙，请运行:"
echo "  sudo ufw enable    (UFW)"
echo "  或"
echo "  sudo systemctl start firewalld    (firewalld)"
echo ""
