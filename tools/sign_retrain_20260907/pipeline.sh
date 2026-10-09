#!/usr/bin/env bash
# Run on the coordinator workstation, after the operator marks data READY.
set -euo pipefail
cd /home/hetaisheng/jetson-nano
mkdir -p field_data/sign_retrain_20260914
exec 9>field_data/sign_retrain_20260914/pipeline.lock
flock -n 9
test ! -f field_data/sign_retrain_20260914/INSTALLED.json
ssh -o BatchMode=yes nano@100.86.37.124 'test -f /home/nano/robodata/signs_20260914/CAPTURE_COMPLETE.json && python2 /home/nano/robocup_ws/tools/sign_retrain_20260907/guided_collect.py --check-ready'
rsync -az --exclude='__pycache__' --exclude='*.pyc' tools/sign_retrain_20260907/ 4090:/home/hts/robodata/tools/
# Use a fresh snapshot: never combine a previous failed transfer with edited data.
run_id=${SIGN_RUN_ID:-$(date +%Y%m%d_%H%M%S)}
[[ "$run_id" =~ ^[0-9]{8}_[0-9]{6}$ ]]
# The repository is an SSHFS mount of Nano. Keep the data snapshot on local ext4.
snapshot="/home/hetaisheng/robodata/runs/$run_id"
mkdir -p "$snapshot/data" "$snapshot/model"
rsync -az nano@100.86.37.124:/home/nano/robodata/signs_20260914/ "$snapshot/data/"
python3 tools/sign_retrain_20260907/guided_collect.py --root "$snapshot/data" --check-ready
ssh 4090 "mkdir -p /home/hts/robodata/runs/$run_id/data"
rsync -az "$snapshot/data/" "4090:/home/hts/robodata/runs/$run_id/data/"
ssh 4090 "/home/hts/robodata/venv/bin/python /home/hts/robodata/tools/guided_collect.py --root /home/hts/robodata/runs/$run_id/data --check-ready"
ssh 4090 "/home/hts/robodata/venv/bin/python -u /home/hts/robodata/tools/train.py --data /home/hts/robodata/runs/$run_id/data --out /home/hts/robodata/runs/$run_id/model --check-data-only"
# Select the currently least occupied GPU; do not stop any other workloads.
gpu=$(ssh 4090 'nvidia-smi --query-gpu=index,memory.free --format=csv,noheader,nounits' | sort -t, -k2,2nr | head -1 | cut -d, -f1 | tr -d ' ')
[[ "$gpu" =~ ^[0-9]+$ ]]
ssh 4090 "CUDA_VISIBLE_DEVICES=$gpu /home/hts/robodata/venv/bin/python -u /home/hts/robodata/tools/train.py --data /home/hts/robodata/runs/$run_id/data --out /home/hts/robodata/runs/$run_id/model --epochs 100 --batch-size 32 > /home/hts/robodata/runs/$run_id/train.log 2>&1"
rsync -az "4090:/home/hts/robodata/runs/$run_id/model/" "$snapshot/model/"
rsync -az "4090:/home/hts/robodata/runs/$run_id/train.log" "$snapshot/"
ssh nano@100.86.37.124 "mkdir -p /home/nano/robodata/models/$run_id /home/nano/robodata/eval/$run_id"
rsync -az "$snapshot/model/" "nano@100.86.37.124:/home/nano/robodata/models/$run_id/"
rsync -az "$snapshot/data/test/" "nano@100.86.37.124:/home/nano/robodata/eval/$run_id/test/"
rsync -az tools/sign_retrain_20260907/verify_replace.py nano@100.86.37.124:/home/nano/robocup_ws/tools/sign_retrain_20260907/
ssh nano@100.86.37.124 "python2 /home/nano/robocup_ws/tools/sign_retrain_20260907/verify_replace.py --candidate /home/nano/robodata/models/$run_id/resnet18_candidate.onnx --data /home/nano/robodata/eval/$run_id --split test --install --yes"
rsync -az "nano@100.86.37.124:/home/nano/robodata/models/$run_id/resnet18_candidate.onnx.nano_eval.json" "$snapshot/model/"
# Keep the workstation framework consistent with the Nano after verified installation.
if ! cmp -s "$snapshot/model/resnet18_candidate.onnx" src/ros/signs/scripts/resnet18.onnx; then
  cp -p src/ros/signs/scripts/resnet18.onnx "$snapshot/model/workstation_previous.onnx"
  cp "$snapshot/model/resnet18_candidate.onnx" src/ros/signs/scripts/.resnet18.new.onnx
  mv src/ros/signs/scripts/.resnet18.new.onnx src/ros/signs/scripts/resnet18.onnx
fi
cp "$snapshot/model/resnet18_candidate.onnx.nano_eval.json" field_data/sign_retrain_20260914/INSTALLED.json
printf 'Installed successfully. Evidence: %s/model\n' "$snapshot"
