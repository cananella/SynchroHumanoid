#!/bin/bash

# FR5 Dataset Upload Script
# Edit the variables below to configure your upload

# ==================== CONFIGURATION ====================
# Dataset path (local directory)
DATASET_PATH="/home/stream/vla/datasets/fr5_test_dataset6"

# HuggingFace repository ID (format: username/dataset_name)
REPO_ID="xhaka3456/fr5_test_dataset6"
# =======================================================

set -e  # Exit on error

# Color codes
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Load HuggingFace token from .env.local
ENV_FILE="/home/stream/vla/.env.local"

if [ ! -f "$ENV_FILE" ]; then
    echo -e "${RED}❌ Error: .env.local file not found at $ENV_FILE${NC}"
    exit 1
fi

# Extract token from .env.local
HF_TOKEN=$(grep "^HUGGINGFACE_TOKEN=" "$ENV_FILE" | cut -d '=' -f2)

if [ -z "$HF_TOKEN" ]; then
    echo -e "${RED}❌ Error: HUGGINGFACE_TOKEN not found in $ENV_FILE${NC}"
    exit 1
fi

# Check if dataset path exists
if [ ! -d "$DATASET_PATH" ]; then
    echo -e "${RED}❌ Error: Dataset path does not exist: $DATASET_PATH${NC}"
    exit 1
fi

# Check if meta/info.json exists
if [ ! -f "$DATASET_PATH/meta/info.json" ]; then
    echo -e "${RED}❌ Error: Not a valid LeRobot dataset (meta/info.json not found)${NC}"
    exit 1
fi

echo "========================================================================"
echo "📤 HuggingFace Dataset Upload"
echo "========================================================================"
echo -e "${YELLOW}Dataset Path:${NC} $DATASET_PATH"
echo -e "${YELLOW}Repository:${NC}   $REPO_ID"
echo ""

# Show dataset info
if [ -f "$DATASET_PATH/meta/info.json" ]; then
    EPISODES=$(grep -o '"total_episodes": [0-9]*' "$DATASET_PATH/meta/info.json" | grep -o '[0-9]*')
    FRAMES=$(grep -o '"total_frames": [0-9]*' "$DATASET_PATH/meta/info.json" | grep -o '[0-9]*')
    echo -e "${GREEN}📊 Episodes:${NC} $EPISODES"
    echo -e "${GREEN}📊 Frames:${NC}   $FRAMES"
    echo ""
fi

# Confirm upload
echo -e "${YELLOW}⚠️  This will upload the dataset to HuggingFace Hub.${NC}"
read -p "Continue? (y/N): " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo -e "${YELLOW}❌ Upload cancelled.${NC}"
    exit 0
fi

echo ""
echo "========================================================================"
echo "🔐 Logging in to HuggingFace..."
echo "========================================================================"

# Login using token
export HUGGING_FACE_HUB_TOKEN="$HF_TOKEN"
huggingface-cli login --token "$HF_TOKEN" --add-to-git-credential

echo ""
echo "========================================================================"
echo "📤 Uploading dataset..."
echo "========================================================================"

# Upload dataset
huggingface-cli upload "$REPO_ID" "$DATASET_PATH" --repo-type=dataset

echo ""
echo "========================================================================"
echo -e "${GREEN}✅ Upload completed!${NC}"
echo "========================================================================"
echo -e "${GREEN}📦 Repository:${NC} https://huggingface.co/datasets/$REPO_ID"
echo "========================================================================"
