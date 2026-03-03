#!/bin/bash
# Install Chinese and Emoji fonts for robot UI

echo "Installing fonts for Robot UI..."

# Update package list
sudo apt-get update

# Install Chinese fonts
echo "Installing Chinese fonts..."
sudo apt-get install -y fonts-noto-cjk fonts-noto-cjk-extra

# Install Emoji fonts
echo "Installing Emoji fonts..."
sudo apt-get install -y fonts-noto-color-emoji

# Optional: Install additional fonts
echo "Installing additional fonts..."
sudo apt-get install -y fonts-wqy-zenhei fonts-wqy-microhei

# Update font cache
echo "Updating font cache..."
fc-cache -f -v

echo ""
echo "✓ Font installation complete!"
echo ""
echo "Available Chinese fonts:"
fc-list :lang=zh | cut -d: -f2 | sort -u | head -5
echo ""
echo "Available Emoji fonts:"
fc-list :charset=1f4f1 | cut -d: -f2 | sort -u
echo ""
echo "You can now run the robot UI:"
echo "  python3 robot_ui.py"
