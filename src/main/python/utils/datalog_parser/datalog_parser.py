#! /usr/bin/env python3
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

import array
import struct
from typing import List, SupportsBytes

import msgpack
from tomlkit import value

import mmap
import sys
from datetime import datetime

import glob
import os

import pandas as pd
import numpy as np

__all__ = ["StartRecordData", "MetadataRecordData", "DataLogRecord", "DataLogReader"]

floatStruct = struct.Struct("<f")
doubleStruct = struct.Struct("<d")

kControlStart = 0
kControlFinish = 1
kControlSetMetadata = 2


class StartRecordData:
    """Data contained in a start control record as created by DataLog.start() when
    writing the log. This can be read by calling DataLogRecord.getStartData().

    entry: Entry ID; this will be used for this entry in future records.
    name: Entry name.
    type: Type of the stored data for this entry, as a string, e.g. "double".
    metadata: Initial metadata.
    """

    def __init__(self, entry: int, name: str, type: str, metadata: str):
        self.entry = entry
        self.name = name
        self.type = type
        self.metadata = metadata


class MetadataRecordData:
    """Data contained in a set metadata control record as created by
    DataLog.setMetadata(). This can be read by calling
    DataLogRecord.getSetMetadataData().

    entry: Entry ID.
    metadata: New metadata for the entry.
    """

    def __init__(self, entry: int, metadata: str):
        self.entry = entry
        self.metadata = metadata


class DataLogRecord:
    """A record in the data log. May represent either a control record
    (entry == 0) or a data record."""

    def __init__(self, entry: int, timestamp: int, data: SupportsBytes):
        self.entry = entry
        self.timestamp = timestamp
        self.data = data

    def isControl(self) -> bool:
        return self.entry == 0

    def _getControlType(self) -> int:
        return self.data[0]

    def isStart(self) -> bool:
        return (
            self.entry == 0
            and len(self.data) >= 17
            and self._getControlType() == kControlStart
        )

    def isFinish(self) -> bool:
        return (
            self.entry == 0
            and len(self.data) == 5
            and self._getControlType() == kControlFinish
        )

    def isSetMetadata(self) -> bool:
        return (
            self.entry == 0
            and len(self.data) >= 9
            and self._getControlType() == kControlSetMetadata
        )

    def getStartData(self) -> StartRecordData:
        if not self.isStart():
            raise TypeError("not a start record")
        entry = int.from_bytes(self.data[1:5], byteorder="little", signed=False)
        name, pos = self._readInnerString(5)
        type, pos = self._readInnerString(pos)
        metadata = self._readInnerString(pos)[0]
        return StartRecordData(entry, name, type, metadata)

    def getFinishEntry(self) -> int:
        if not self.isFinish():
            raise TypeError("not a finish record")
        return int.from_bytes(self.data[1:5], byteorder="little", signed=False)

    def getSetMetadataData(self) -> MetadataRecordData:
        if not self.isSetMetadata():
            raise TypeError("not a finish record")
        entry = int.from_bytes(self.data[1:5], byteorder="little", signed=False)
        metadata = self._readInnerString(5)[0]
        return MetadataRecordData(entry, metadata)

    def getBoolean(self) -> bool:
        if len(self.data) != 1:
            raise TypeError("not a boolean")
        return self.data[0] != 0

    def getInteger(self) -> int:
        if len(self.data) != 8:
            raise TypeError("not an integer")
        return int.from_bytes(self.data, byteorder="little", signed=True)

    def getFloat(self) -> float:
        if len(self.data) != 4:
            raise TypeError("not a float")
        return floatStruct.unpack(self.data)[0]

    def getDouble(self) -> float:
        if len(self.data) != 8:
            raise TypeError("not a double")
        return doubleStruct.unpack(self.data)[0]

    def getString(self) -> str:
        return str(self.data, encoding="utf-8")

    def getMsgPack(self):
        return msgpack.unpackb(self.data)

    def getBooleanArray(self) -> List[bool]:
        return [x != 0 for x in self.data]

    def getIntegerArray(self) -> array.array:
        if (len(self.data) % 8) != 0:
            raise TypeError("not an integer array")
        arr = array.array("l")
        arr.frombytes(self.data)
        return arr

    def getFloatArray(self) -> array.array:
        if (len(self.data) % 4) != 0:
            raise TypeError("not a float array")
        arr = array.array("f")
        arr.frombytes(self.data)
        return arr

    def getDoubleArray(self) -> array.array:
        if (len(self.data) % 8) != 0:
            raise TypeError("not a double array")
        arr = array.array("d")
        arr.frombytes(self.data)
        return arr

    def getStringArray(self) -> List[str]:
        size = int.from_bytes(self.data[:4], byteorder="little", signed=False)
        if size > ((len(self.data) - 4) / 4):
            raise TypeError("not a string array")
        arr = []
        pos = 4
        for _ in range(size):
            val, pos = self._readInnerString(pos)
            arr.append(val)
        return arr

    def _readInnerString(self, pos: int) -> tuple[str, int]:
        size = int.from_bytes(
            self.data[pos : pos + 4], byteorder="little", signed=False
        )
        end = pos + 4 + size
        if end > len(self.data):
            raise TypeError("invalid string size")
        return str(self.data[pos + 4 : end], encoding="utf-8"), end


class DataLogIterator:
    """DataLogReader iterator."""

    def __init__(self, buf: SupportsBytes, pos: int):
        self.buf = buf
        self.pos = pos

    def __iter__(self):
        return self

    def _readVarInt(self, pos: int, len: int) -> int:
        val = 0
        for i in range(len):
            val |= self.buf[pos + i] << (i * 8)
        return val

    def __next__(self) -> DataLogRecord:
        if len(self.buf) < (self.pos + 4):
            raise StopIteration
        entryLen = (self.buf[self.pos] & 0x3) + 1
        sizeLen = ((self.buf[self.pos] >> 2) & 0x3) + 1
        timestampLen = ((self.buf[self.pos] >> 4) & 0x7) + 1
        headerLen = 1 + entryLen + sizeLen + timestampLen
        if len(self.buf) < (self.pos + headerLen):
            raise StopIteration
        entry = self._readVarInt(self.pos + 1, entryLen)
        size = self._readVarInt(self.pos + 1 + entryLen, sizeLen)
        timestamp = self._readVarInt(self.pos + 1 + entryLen + sizeLen, timestampLen)
        if len(self.buf) < (self.pos + headerLen + size):
            raise StopIteration
        record = DataLogRecord(
            entry,
            timestamp,
            self.buf[self.pos + headerLen : self.pos + headerLen + size],
        )
        self.pos += headerLen + size
        return record


class DataLogReader:
    """Data log reader (reads logs written by the DataLog class)."""

    def __init__(self, buf: SupportsBytes):
        self.buf = buf

    def __bool__(self):
        return self.isValid()

    def isValid(self) -> bool:
        """Returns true if the data log is valid (e.g. has a valid header)."""
        return (
            len(self.buf) >= 12
            and self.buf[:6] == b"WPILOG"
            and self.getVersion() >= 0x0100
        )

    def getVersion(self) -> int:
        """Gets the data log version. Returns 0 if data log is invalid.

        @return Version number; most significant byte is major, least significant is
            minor (so version 1.0 will be 0x0100)"""
        if len(self.buf) < 12:
            return 0
        return int.from_bytes(self.buf[6:8], byteorder="little", signed=False)

    def getExtraHeader(self) -> str:
        """Gets the extra header data.

        @return Extra header data
        """
        if len(self.buf) < 12:
            return ""
        size = int.from_bytes(self.buf[8:12], byteorder="little", signed=False)
        return str(self.buf[12 : 12 + size], encoding="utf-8")

    def __iter__(self) -> DataLogIterator:
        extraHeaderSize = int.from_bytes(
            self.buf[8:12], byteorder="little", signed=False
        )
        return DataLogIterator(self.buf, 12 + extraHeaderSize)

def generate_header(num_columns, delimiter=","):
    # Create the base columns
    header_list = ["n", "type", "entry", "size", "name", "entry_type", "timestamp"]
    
    # Generate columns based on input number (0 to num_columns-1)
    for i in range(num_columns):
        header_list.append(f"data{i}")
        
    # Join list into a single comma-separated string
    return delimiter.join(header_list)

# Function to parse datalog files and generate CSVs based on patterns
def parse_datalog(fname_in, fname_out, match_patterns):
    n_value = 0
    highest_index = 0
    lines_buffer = []

    with open(fname_in, "r") as f, open(fname_out, 'w') as w:
        mm = mmap.mmap(f.fileno(), 0, access=mmap.ACCESS_READ)
        reader = DataLogReader(mm)
        if not reader:
            print("not a log file", file=sys.stderr)
            sys.exit(1)

        entries = {}
        for record in reader:
            this_index = 0
            timestamp = record.timestamp / 1000000
            if record.isStart():
                try:
                    data = record.getStartData()
                    #print(
                    #    f"Start({data.entry}, name='{data.name}', type='{data.type}', metadata='{data.metadata}') [{timestamp}]"
                    #)
                    if data.entry in entries:
                        print("...DUPLICATE entry ID, overriding")
                    entries[data.entry] = data
                except TypeError:
                    print("Start(INVALID)")
            elif record.isFinish():
                try:
                    entry = record.getFinishEntry()
                    print(f"Finish({entry}) [{timestamp}]")
                    if entry not in entries:
                        print("...ID not found")
                    else:
                        del entries[entry]
                except TypeError:
                    print("Finish(INVALID)")
            elif record.isSetMetadata():
                try:
                    data = record.getSetMetadataData()
                    print(f"SetMetadata({data.entry}, '{data.metadata}') [{timestamp}]")
                    if data.entry not in entries:
                        print("...ID not found")
                except TypeError:
                    print("SetMetadata(INVALID)")
            elif record.isControl():
                print("Unrecognized control record")
            else:
                # Save the current string for string data
                current_string = f"{n_value},Data,{record.entry},{len(record.data)}"
                n_value = n_value + 1
                #print(f"{current_string}", end="")
                entry = entries.get(record.entry)
                if entry is None:
                    print("<ID not found>")
                    continue
                current_string = f"{current_string},{entry.name},{entry.type},{timestamp}"
                
                # if the entry name contains any of the match patterns, print the data
                if any(match_pattern in entry.name for match_pattern in match_patterns):
                    #print(f"{current_string}", end="")

                    try:
                        # handle systemTime specially
                        if entry.name == "systemTime" and entry.type == "int64":
                            dt = datetime.fromtimestamp(record.getInteger() / 1000000)
                            print(", {:%Y-%m-%d %H:%M:%S.%f}".format(dt), end="")
                            continue

                        if entry.type == "double":
                            highest_index = max(highest_index, 1)
                            #print(f", {record.getDouble()}")
                            current_string = f"{current_string},{record.getDouble()}"

                        elif entry.type == "int64":
                            highest_index = max(highest_index, 1)                            
                            #print(f", {record.getInteger()}")
                            current_string = f"{current_string},{record.getInteger()}"

                        elif entry.type in ("string", "json"):
                            highest_index = max(highest_index, 1)                            
                            #print(f", '{record.getString()}'")
                            current_string = f"{current_string},'{record.getString()}'"

                        elif entry.type == "msgpack":
                            highest_index = max(highest_index, 1)                            
                            #print(f", '{record.getMsgPack()}'")
                            current_string = f"{current_string},'{record.getMsgPack()}'"

                        elif entry.type == "boolean":
                            highest_index = max(highest_index, 1)                            
                            #print(f", {record.getBoolean()}")
                            current_string = f"{current_string},{record.getBoolean()}"

                        elif entry.type == "boolean[]":
                            arr = record.getBooleanArray()
                            #print(f", {arr}")
                            for item in arr:
                                this_index += 1                                
                                #print(f", {item}", end="")
                                current_string = f"{current_string},{item}"
                                 
                            highest_index = max(highest_index, this_index)                       

                        elif entry.type == "double[]":
                            arr = record.getDoubleArray()
                            for item in arr:
                                this_index += 1
                                #print(f", {item}", end="")
                                current_string = f"{current_string},{item}"
                            
                            highest_index = max(highest_index, this_index)                                                   
                            #print(f", {list(arr)}")

                        elif entry.type == "float":
                            highest_index = max(highest_index, 1)                            
                            #print(f", {record.getFloat()}")
                            current_string = f"{current_string},{record.getFloat()}"

                        elif entry.type == "float[]":
                            arr = record.getFloatArray()
                            for item in arr:
                                this_index += 1
                                #print(f", {item}", end="")
                                current_string = f"{current_string},{item}"
                            #print(f", {arr}")

                        elif entry.type == "int64[]":
                            arr = record.getIntegerArray()
                            for item in arr:
                                this_index += 1
                                #print(f", {item}", end="")
                                current_string = f"{current_string},{item}"
                            
                            highest_index = max(highest_index, this_index)                                                   
                            #print(f", {arr}")

                        elif entry.type == "string[]":
                            arr = record.getStringArray()
                            #print(f", {arr}")
                            current_string = f"{current_string},{arr}"

                        elif entry.type == "raw":
                            highest_index = max(highest_index, 1)                            
                            arr = record.getStringArray()
                            #print(f", RAW")
                            current_string = f"{current_string},'{record.getStringArray()}'"  

                        elif entry.type == "URCLr3_periodic":
                            highest_index = max(highest_index, 1)                            
                            #print(f", URCL")
                            current_string = f"{current_string},URCL"

                        elif "struct" in entry.type:
                            highest_index = max(highest_index, 1)                            
                            #print(f", STRUCT")
                            current_string = f"{current_string},STRUCT"

                        else:
                            highest_index = max(highest_index, 1)                            
                            #print(f", UNKNOWN")
                            current_string = f"{current_string},UNKNOWN"

                        # Write current string to file
                        lines_buffer.append(current_string)

                    except TypeError:
                        print(", invalid")

    # Write the line buffer data to a file with the generated header
    with open(fname_out, 'w') as w:
        #print(highest_index)
        header_string = generate_header(highest_index)
        #print(header_string)
        w.write(header_string + "\n")
        for line in lines_buffer:
            w.write(line + "\n")
            w.flush()
        
# Function to calculate trapezoidal sums for each column in the generated CSV file
def calc_trapezoidal_sums(csv_filename):
   df = pd.read_csv(csv_filename)
   # print(df)
   x_values = df.iloc[:,6].values
   trapezoidal_sums = {}
   for column_name in df.columns[7:]:
      # print(f"Working on {column_name}")
      y_values = df[column_name].values
      trapz_sum = np.trapezoid(y_values, x=x_values)
      trapezoidal_sums[column_name] = trapz_sum
   return trapezoidal_sums

def process_ChannelCurrents(wpilog_files, cfg_directory, input_directory, output_directory):
    # Create an empty sums_df to hold the trapezoidal sums for each pattern and file and set index name to match_pattern
    sums_df = pd.DataFrame()
    sums_df.index.name = 'match_pattern'    

    # Parse all wpilog files and generate corresponding CSV files for the list of patterns
    for wpilog_file in wpilog_files:
        print(f"Processing file: {wpilog_file}")

        fname_in = wpilog_file

        # Generate output filename by replacing .wpilog with .csv and placing it in the output directory
        fname_ext_out = f".ChannelCurrent.csv"
        fname_out = f"{output_directory}/{os.path.basename(wpilog_file).replace('.wpilog', fname_ext_out)}"
        print(f"Output file will be: {fname_out}")

        channelcurrent_patterns = ['ChannelCurrent']

        print(f"Matching patterns: {channelcurrent_patterns}")
        parse_datalog(fname_in=fname_in, fname_out=fname_out, match_patterns=channelcurrent_patterns)
        print(f"Finished processing patterns: {channelcurrent_patterns} for file: {wpilog_file}")

        # Calculate trapezoidal sums for the generated CSV file
        sums = calc_trapezoidal_sums(fname_out)

        # Generate the column name for this dataset by taking the base filename and removing the .wpilog extension
        column_name = os.path.basename(wpilog_file).replace('.wpilog', '')
        # Add the sums to the sums_df with the column name as the index
        sums_df[column_name] = pd.Series(sums)

    # Read in map file for ChannelCurrents to get the mapping of dataX to their respective names
    map_file = f"{cfg_directory}/map-ChannelCurrent.txt"

    # Read in the map file
    map_col_names = ['data_name', 'mapping']
    map_df = pd.read_csv(map_file, header=None, names=map_col_names)

    # Create a full name column
    map_df['full_name'] = map_df['data_name'] + "_" + map_df['mapping']

    # Rename the index of sums_df to match the full_name column in map_df
    sums_df.index = map_df['full_name']

    # Set the sum in seconds to zero
    sums_df.loc['sumSecs'] = 0

    # for each full_name field in map_df, add that data to the numSecs sum in sums_df
    #for index, row in map_df.iterrows():
    #    full_name = row['full_name']
    #    if full_name in sums_df.index:
    #        sums_df.loc['sumSecs'] += sums_df.loc[full_name]

    # Sum all rows into sumSecs row
    sums_df.loc['sumSecs'] = sums_df.sum()
    sums_df.loc['sumMins'] = sums_df.loc['sumSecs'] / 60
    sums_df.loc['sumHours'] = sums_df.loc['sumMins'] / 60

    sums_output_file = f"{output_directory}/ChannelCurrent_trapezoidal_sums.csv"
    sums_df.to_csv(sums_output_file)
    print(f"Trapezoidal sums saved to: {sums_output_file}")    

# Function to calculate trapezoidal sums for each column in the generated CSV file
def calc_camera_sums(csv_filename):
    df_cam = pd.read_csv(csv_filename)
    print(f"Processing Camera Data from file: {csv_filename}")

    # First find when DriverStation/Enabled is true and false and calculate the time difference between each change in state
    df_enabled = df_cam[df_cam['name'].str.contains('DriverStation/Enabled')]
    df_enabled['time_diff'] = df_enabled['timestamp'].diff().fillna(0)

    # Sum the total enabled time
    total_enabled_time = round(df_enabled[df_enabled['data0'] == "True"]['time_diff'].sum(), 2)
    # Print the total enabled time
    print(f"Total Enabled Time: {total_enabled_time} seconds")

    for i in range(4):
        cam_pattern = f"Camera{i}/Connected"
        df_cam_data = df_cam[df_cam['name'].str.contains(cam_pattern)]
        df_cam_data['time_diff'] = df_cam_data['timestamp'].diff().fillna(0)
        df_cam_data['true_time'] = df_cam_data.apply(lambda row: row['time_diff'] if row['data0'] == "True" else 0, axis=1)
        df_cam_data['false_time'] = df_cam_data.apply(lambda row: row['time_diff'] if row['data0'] == "False" else 0, axis=1)
        total_true_time = round(df_cam_data['true_time'].sum(), 2)
        total_false_time = round(df_cam_data['false_time'].sum(), 2)
        print(f"{cam_pattern}: Total True Time: {total_true_time} seconds, Total False Time: {total_false_time} seconds")

def process_CameraConnected(wpilog_files, cfg_directory, input_directory, output_directory):
    # Parse all wpilog files and generate corresponding CSV files for the list of patterns
    for wpilog_file in wpilog_files:
        print(f"Processing file: {wpilog_file}")

        fname_in = wpilog_file

        # Generate output filename by replacing .wpilog with .csv and placing it in the output directory
        fname_ext_out = f".CameraConnected.csv"
        fname_out = f"{output_directory}/{os.path.basename(wpilog_file).replace('.wpilog', fname_ext_out)}"
        print(f"Output file will be: {fname_out}")

        camera_patterns = ['Camera0/Connected', 'Camera1/Connected', 'Camera2/Connected', 'Camera3/Connected', 'DriverStation/MatchTime', "DriverStation/Enabled", 'DriverStation/Autonomous']

        print(f"Matching patterns: {camera_patterns}")
        parse_datalog(fname_in=fname_in, fname_out=fname_out, match_patterns=camera_patterns)
        print(f"Finished processing patterns: {camera_patterns} for file: {wpilog_file}")

        calc_camera_sums(fname_out)


# Main function to parse datalog files and generate CSVs based on patterns
if __name__ == "__main__":
    if len(sys.argv) != 4:
        print(f"Usage: {sys.argv[0]} <cfg_directory> <input_directory> <output_directory>", file=sys.stderr)
        sys.exit(1)

    cfg_directory = sys.argv[1]
    input_directory = sys.argv[2]
    output_directory = sys.argv[3]
    wpilog_patterns_file = f"{cfg_directory}/wpilog_patterns.txt"

    # Get a list of all wpilog files in the specified directory
    wpilog_files = glob.glob(f"{input_directory}/*.wpilog")

    # Add each line of the wpilog patterns file to a list
    match_patterns = []
    with open(wpilog_patterns_file, 'r') as f:
        for line in f:
            match_patterns.append(line.strip())

    # For all patterns
    for match_pattern in match_patterns:
        if match_pattern == "ChannelCurrent":
            process_ChannelCurrents(wpilog_files, cfg_directory, input_directory, output_directory)
        elif match_pattern == "CameraConnected":
            process_CameraConnected(wpilog_files, cfg_directory, input_directory, output_directory)
        else:
            print(f"Pattern {match_pattern} not recognized, skipping.")
            