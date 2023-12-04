#!/bin/bash

# script takes a list of numbers to generate directories for corresponding octoprint servers
# helpful links
# https://github.com/OctoPrint/OctoPrint/tree/master/scripts
# http://thomas-messmer.com/index.php/14-free-knowledge/howtos/79-setting-up-octoprint-for-multiple-printers

# Default values for options

id_numbers=()
create_servers=false
delete_servers=false

usage() {
    echo "Use this script to manage OctoPrint server instances. A single OctoPrint server instance can only control one smoothie board, which can subsequently control up to 2 fluid input sources."
    echo "Unless you know what you're doing, do not attempt to create OctoPrint server instances outside of this script to avoid introducing errors."
    echo "run as sudo"
    echo "Options:"
    echo "  -i <identification_numbers>    Create OctoPrint server instances based on ID input list (space-separated)"
    echo "  -d <identification_numbers>    Delete OctoPrint server instances based on ID input list (space-separated)"
    echo "  -l                             List current OctoPrint servers"
    echo "  -v                             Verbose"

    echo "Examples:"
    echo "create OctoPrint server instaces: $0 -i 2 3 4 "
    echo "delete OctoPrint server instaces: $0 -d 3 4 "
    echo "list OctoPrint server instances: $0 -l"
}

delete(){
    # Iterate through the identification numbers and delete existing directories
    for id in "$@"; do
        packageDir="/home/pi/.octoprint$id"
        OctoPrintDir="/home/pi/OctoPrint$id"
        initPath="/etc/init.d/octoprint$id"
        defaultPath="/etc/default/octoprint$id"

        directories=("$packageDir" "$OctoPrintDir")
        files=("$initPath" "$defaultPath")

    # Check if the directory already exists, if so delete 
        for dir in "${directories[@]}"; do
            if [ -d "$dir" ]; then
                echo "Directory $dir already exists. Deleting ..."
                rm -r "$dir"
            fi
        done

        for file in "${files[@]}"; do
            if [ -f "$file" ]; then
                echo "File $file already exists. Deleting  ..."
                rm -f "$file"
            fi
        done
    done
}

# Parse command line options
while getopts ":i:d:lv" opt; do
    case $opt in
        i)
            id_numbers=($OPTARG)
            create_servers=true
            ;;
        d)
            id_numbers=($OPTARG)
            delete_servers=true
            ;;
        l)
            list_servers=true
            ;;
        v)
            usage
            exit 1
            ;;
        \?)
            echo "Invalid option: -$OPTARG" >&2
            usage
            exit 1
            ;;
        :)
            echo "Option -$OPTARG requires an argument." >&2
            usage
            exit 1
            ;;
    esac
done

# Check if at least one argument is provided
if [ "$create_servers" = true ]; then
    delete "$id_numbers"
    for id in "${id_numbers[@]}"; do
        packageDir="/home/pi/.octoprint$id"
        OctoPrintDir="/home/pi/OctoPrint$id"
        initPath="/etc/init.d/octoprint$id"
        defaultPath="/etc/default/octoprint$id"

        directories=("$packageDir" "$OctoPrintDir")
        files=("$initPath" "$defaultPath")

        # Copy directories from backup to make necessary server directories and files
        cp -r "./octoprint_backup_directories/.octoprint.bck" "$packageDir"
        cp -r "./octoprint_backup_directories/OctoPrint.bck" "$OctoPrintDir"
        cp "./octoprint_backup_files/octoprint_default.bck" "$defaultPath"
        cp "./octoprint_backup_files/octoprint_init.bck" "$initPath"

        # Search and replace instances of "x.bck" with "x.id" in server files 
        search_phrase_1="OctoPrint.bck"
        search_phrase_2="octoprint.bck"
        replace_phrase_1="OctoPrint$id"
        replace_phrase_2="octoprint$id"

        file_paths=("$initPath" "$defaultPath")
        for file_path in "${file_paths[@]}"; do
            sed -i "s/$search_phrase_1/$replace_phrase_1/g" "$file_path"
            sed -i "s/$search_phrase_2/$replace_phrase_2/g" "$file_path"
        done

        # Setup port server uses (5000 + id)
        port=$((5000 + id))
        sed -i "s/5000/$port/g" "$defaultPath" 
        # Enable octoprint service
        sudo update-rc.d octoprint$id defaults
    done
fi

if [ "$delete_servers" = true ]; then
    delete "$id_numbers"
fi

if [ "$list_servers" = true ]; then
    # Get the substring to search for
    substring="OctoPrint"

    # Find all directories whose names contain the substring
    echo "List of OctoPrint server instances:"
    find "/home/pi/"  -maxdepth 2 -type d -name "*${substring}*" -print
fi