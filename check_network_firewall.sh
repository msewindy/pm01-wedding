#!/bin/bash
# 网络和防火墙状态检查脚本

echo "=========================================="
echo "网络和防火墙状态检查"
echo "=========================================="

# 1. 防火墙状态
echo ""
echo "1. 防火墙状态:"
if command -v ufw &> /dev/null; then
    echo "   - UFW状态:"
    ufw status 2>/dev/null | sed 's/^/     /' || echo "     需要sudo权限查看详细状态"
else
    echo "   - UFW未安装"
fi

if command -v firewall-cmd &> /dev/null; then
    echo "   - firewalld状态:"
    firewall-cmd --state 2>/dev/null | sed 's/^/     /' || echo "     未运行"
else
    echo "   - firewalld未安装"
fi

# 2. 网络接口
echo ""
echo "2. 网络接口:"
ip -br addr show | sed 's/^/   /'

# 3. 路由表
echo ""
echo "3. 路由表:"
ip route show | sed 's/^/   /'

# 4. DNS配置
echo ""
echo "4. DNS配置:"
if command -v resolvectl &> /dev/null; then
    echo "   - 当前DNS服务器:"
    resolvectl status | grep "DNS Server" | head -1 | sed 's/^/     /' || echo "     未找到"
else
    echo "   - resolvectl未安装"
fi
echo "   - /etc/resolv.conf:"
cat /etc/resolv.conf | grep -v "^#" | grep -v "^$" | sed 's/^/     /'

# 5. 网络内核参数
echo ""
echo "5. 关键网络内核参数:"
echo "   - IP转发 (ip_forward): $(cat /proc/sys/net/ipv4/ip_forward)"
echo "   - IPv6转发: $(cat /proc/sys/net/ipv6/conf/all/forwarding 2>/dev/null || echo 'N/A')"

# 6. NetworkManager状态
echo ""
echo "6. NetworkManager状态:"
if command -v nmcli &> /dev/null; then
    echo "   - 活动连接:"
    nmcli connection show --active | sed 's/^/     /'
else
    echo "   - NetworkManager未安装"
fi

# 7. netplan配置
echo ""
echo "7. netplan配置:"
if [ -f /etc/netplan/01-network-manager-all.yaml ]; then
    echo "   - 配置文件内容:"
    cat /etc/netplan/01-network-manager-all.yaml | sed 's/^/     /'
else
    echo "   - 未找到netplan配置文件"
fi

echo ""
echo "=========================================="
echo "检查完成"
echo "=========================================="
