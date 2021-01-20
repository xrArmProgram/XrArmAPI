find . -name "*.wav" |awk -F "." '{sub(/\//, "");print $2 " = " "audio_dir + " "\""$2 ".wav" "\"";}' >> genrate.py
