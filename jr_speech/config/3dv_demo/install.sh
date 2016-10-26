#!/bin/bash

vocab=$1

if [[ -n "$vocab" ]]; then
    /usr/bin/rename -f "s/$vocab/3dv_commands/" "$vocab".*
else
    echo "Missing vocab number."
fi

