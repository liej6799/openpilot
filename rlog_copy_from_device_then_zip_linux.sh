#!/usr/bin/env bash

# Copies all rlog files from your comma device, over LAN or comma SSH, 
# to the specified local directory.
# devicehostname must match a defined ssh host setup for passwordless access using ssh keys
# You can prevent redundant transfers in two ways:
# If the to-be-transferred rlog already exists in the destination
# it will not be transferred again, so you can use the same directory
# and leave the files there

# List of ssh hosts and car names (two words separated by a space)
device_car_list=( 
"c3dave acadia"
"c3local volt"
)
# Time between checks for completed processes.
check_interval=30
# Maximum time between starting fetches for each device.
interval=900

diroutbase="/mnt/video/scratch-video/rlog_api"

check_dir="$diroutbase"/../rlogs
cd "$check_dir"
check_list=$(find . -name "*rlog*")
# echo "$check_list"
int_re='^[0-9]+$'

fetch_rlogs () {
  echo "$1 ($2): Fetching dongle ID"
  devicehostname="$1"
  DONGLEID=`ssh $devicehostname cat /data/params/d/DongleId`
  if [ $? -ne 0 ]; then
    echo "$1 ($2): device not online..."
    return 1
  fi
  dirout="$diroutbase/$2/$DONGLEID"

  # NO TOUCHING
  devicedirin="/data/media/0/realdata"
  i=1
  r=1
  iter=0
  tot=0
  r_old=0
  while [ $i -gt 0 ] || [ $r -ne $r_old ]; do
    isoffroad=`ssh $devicehostname cat /data/params/d/IsOffroad`
    if ! [[ "$isoffroad" =~ $int_re ]]; then
      echo "$1 ($2): skipping: *** FAILED TO CHECK IF ONROAD ***"
      return
    fi
    if [ "$isoffroad" -ne 1 ]; then
      echo "$1 ($2): skipping: *** DEVICE IS ONROAD ***"
      return
    fi
    r_old=$r
    i=0
    r=0
    skipped=0
    iter=$((iter + 1))

    echo "$1 ($2): Starting copy of rlogs from device (dongleid $DONGLEID; iteration $iter) to $dirout"

    echo "$1 ($2): Fetching list of candidate files to be transferred"
    # get list of files to be transferred
    remotefilelist=$(ssh $devicehostname "if find / -maxdepth 0 -printf \"\" 2>/dev/null; then
        nice -19 find \"$devicedirin\" -name \"*rlog*\" -printf \"%T@ %Tc ;;%p\n\" | sort -n | sed 's/.*;;//'
      elif stat --version | grep -q 'GNU coreutils'; then
        nice -19 find \"$devicedirin\" -name \"*rlog*\" -exec stat -c \"%Y %y %n\" {} \; | sort -n | cut -d ' ' -f 5-
      else
        echo \"Neither -printf nor GNU coreutils stat is available\" >&2
        exit 1
      fi")
    
    if [ $? -eq 0 ]; then
      mkdir -p "$dirout"
    else
      echo "$1 ($2): $remotefilelist"
      break
    fi
    
    echo "$1 ($2): Check for duplicate files"

    fileliststr="
    "
    for f in $remotefilelist; do
      fstr="${f#$devicedirin/}" # strip off the input directory
      if [[ $fstr == *.bz2 ]]; then
        route="${fstr%%/rlog.bz2}"
      else
        route="${fstr%%/rlog}"
      fi
      ext="${fstr#$route/}"
      lfn="$dirout/$DONGLEID"_"$route"--"$ext"
      lfnbase="$dirout/$DONGLEID"_"$route"--rlog
      
      if [[ "$f" != *.bz2 ]] && [[ -f "$lfnbase".bz2 || "$check_list" == *"$route"* ]] ; then
        skipped=$((skipped+1))
        continue
      elif [[ "$check_list" == *"$route"* ]] || [ -f "$lfnbase" ] || [ -f "$lfnbase".bz2 ]; then
        fileliststr="$fileliststr get -a \"$f\" \"$lfn\"
        "
        r=$((r+1))
      else
        fileliststr="$fileliststr get \"$f\" \"$lfn\"
        "
        i=$((i+1))
      fi
    done
    
    if [ $r -eq $r_old ]; then
      return 0
    fi

    echo "$1 ($2): Total transfers: $((i+r)) = $i new + $r resumed"
    echo "$1 ($2): Skipped transfers: $skipped"
    tot=$((tot + i))

    # perform transfer
    if [[ $i -gt 0  || ( $r -gt 0 && $r -ne $r_old ) ]]; then
      echo "$1 ($2): Beginning transfer"
      sftp -C $devicehostname << EOF 
"$fileliststr"

EOF
      echo "$1 ($2): Transfer complete (returned $?)"
    fi
  done
  
  return 0
}

# Calculate the number of check_intervals in the interval.
interval_count=$((interval / check_interval))

# Associative arrays to store the PIDs and the number of runs of the fetch_rlogs processes.
declare -A device_pid_map
declare -A device_run_count_map

# Initialize the run count map for each device.
for d in "${device_car_list[@]}"; do
  device_run_count_map[$d]=0
done

while true; do
  all_done=true

  for ((i=0; i<interval_count; i++)); do
    for d in "${device_car_list[@]}"; do
      if [ -z "${device_pid_map[$d]}" ] || ! kill -0 "${device_pid_map[$d]}" 2>/dev/null; then
        if [ $i -eq 0 ]; then
          fetch_rlogs $d &
          device_pid_map[$d]=$!
          c=$((device_run_count_map[$d] + 1))
          device_run_count_map[$d]=$c
          echo "Beginning device rlog fetch $c for $d"
        fi
        all_done=false
      fi
    done

    # Sleep for the check_interval.
    sleep $check_interval
  done

  # Check if all fetches have run to completion at least once.
  all_done=true
  for d in "${device_car_list[@]}"; do
    if [ ${device_run_count_map[$d]} -eq 0 ]; then
      all_done=false
      break
    fi
  done

  # Break the outer loop if all fetches have run to completion at least once.
  if $all_done; then
    break
  fi
done

# Wait for all fetch_rlogs processes to finish.
wait

echo "zipping any unzipped rlogs"
# if the following line won't work, you can force it by swapping it with the subsequent line.
find "$diroutbase" -not -path '*/\.*' -type f -name "*rlog" -print -exec bzip2 -f --verbose {} \;
# echo "<your damn admin password>" | sudo -S find "$diroutbase" -not -path '*/\.*' -type f -name "*rlog" -print -exec bzip2 -f --verbose {} \;



echo "Done"

