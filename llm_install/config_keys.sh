#!/bin/bash
# -*- coding: utf-8 -*-
# flake8: noqa
#
# Copyright 2023 Herman Ye @Auromix
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Description:
# This script will add your OpenAI API_KEY to your .bashrc file.
#
# Author: Herman Ye @Auromix

echo "This script will add your OpenAI and ElevenLabs API keys to your .bashrc file."

# Ask user for OpenAI API key
read -rp "Enter your OpenAI API key: " OPENAI_API_KEY

# Ask user for ElevenLabs API key
read -rp "Enter your ElevenLabs API key: " ELEVENLABS_API_KEY

# Function to update or add API keys in .bashrc
update_or_add_key() {
  KEY_NAME=$1
  KEY_VALUE=$2
  if grep -q "export $KEY_NAME" ~/.bashrc; then
    echo "Existing $KEY_NAME found in .bashrc file."
    read -rp "Are you sure you want to replace the existing $KEY_NAME in your .bashrc file? (y/n) " confirm
    if [[ "$confirm" =~ ^[Yy]$ ]]; then
      sed -i "/export $KEY_NAME/d" "$HOME/.bashrc"
      echo -e "\nExisting $KEY_NAME was removed from .bashrc file."
    else
      echo "No changes were made to $KEY_NAME."
      return
    fi
  fi
  # Ensure a new line is added before appending the new key
  echo -e "\nexport $KEY_NAME=$KEY_VALUE" >> "$HOME/.bashrc"
  echo "Added $KEY_NAME=$KEY_VALUE to .bashrc file."
}

# Update or add OpenAI API key
update_or_add_key "OPENAI_API_KEY" "$OPENAI_API_KEY"

# Update or add ElevenLabs API key
update_or_add_key "ELEVENLABS_API_KEY" "$ELEVENLABS_API_KEY"

# Inform the user that they need to reload their .bashrc or restart the terminal
echo "Please close and reopen your terminal or run 'source ~/.bashrc' to apply the changes."

# Wait for user to exit
read -n 1 -r -p "Press any key to exit..."
exit 0
