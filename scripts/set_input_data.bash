#!/usr/bin/env bash

START_DIR=$PWD
if [ "$#" -ne 1 ]; then
  echo "Please select the file you wish to set as [ input_data ]:"
  cd $START_DIR/data
  user_choice=(*)

  declare -i index=1
  for i in ${user_choice[@]}
    do
      echo $index - $i
      index=$(( index + 1 ))
    done
  read ans
  chosen_index=$(( ans - 1 ))

  echo "Setting [ ${user_choice[$chosen_index]} ] as [input_data] "
  echo $PWD
  echo "Running command: 'ln -sf ${user_choice[$chosen_index]} input_data' "
  ln -sf ${user_choice[$chosen_index]} input_data
fi

cd $START_DIR
