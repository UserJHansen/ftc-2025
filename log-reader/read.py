#! /usr/bin/env nix-shell
#! nix-shell -i python3 -p python3Packages.matplotlib python3Packages.requests python3Packages.beautifulsoup4 python3Packages.numpy

import enum
import struct
import typing as T
from dataclasses import dataclass
from collections import defaultdict

import matplotlib.pyplot as plt
from collections import defaultdict

import requests
from bs4 import BeautifulSoup
import io
import numpy as np

MessageSchema = T.Union['StructSchema', 'PrimitiveSchema']

@dataclass(frozen=True)
class StructSchema:
    fields: T.Dict[str, MessageSchema]

class PrimitiveSchema(enum.Enum):
    INT = 'int'
    LONG = 'long'
    DOUBLE = 'double'
    STRING = 'string'
    BOOLEAN = 'boolean'

@dataclass(frozen=True)
class EnumSchema:
    constants: T.List[str]


def read_file(f):
    def read(n):
        assert n > 0
        # assume this reads exactly n bytes or we reach EOF
        buf = f.read(n)
        if len(buf) == 0:
            raise EOFError
        if len(buf) < n:
            raise IOError('Short read')
        return buf

    def read_string():
        nbytes, = struct.unpack_from('!i', read(4))    
        name, = struct.unpack_from(f'!{nbytes}s', read(nbytes))
        return name.decode('utf-8')

    def read_schema():
        schema_type, = struct.unpack_from('!i', read(4))
        # struct schema
        if schema_type == 0:
            nfields, = struct.unpack_from('!i', read(4)) 
            fields = {}
            for _ in range(nfields):
                name = read_string()
                fields[name] = read_schema()
            return StructSchema(fields)
        # primitive schema
        elif schema_type == 1:
            return PrimitiveSchema.INT
        elif schema_type == 2:
            return PrimitiveSchema.LONG
        elif schema_type == 3:
            return PrimitiveSchema.DOUBLE
        elif schema_type == 4:
            return PrimitiveSchema.STRING
        elif schema_type == 5:
            return PrimitiveSchema.BOOLEAN
        # enum schema
        elif schema_type == 6:
            nconstants, = struct.unpack_from('!i', read(4))
            constants = []
            for _ in range(nconstants):
                constants.append(read_string())
            return EnumSchema(constants) 
        else:
            raise ValueError(f'Unknown schema type: {schema_type}')

    def read_msg(schema):
        if isinstance(schema, StructSchema):
            msg = {}
            for name, field_schema in schema.fields.items():
                msg[name] = read_msg(field_schema)
            return msg
        elif isinstance(schema, PrimitiveSchema):
            if schema == PrimitiveSchema.INT:
                return struct.unpack_from('!i', read(4))[0]
            elif schema == PrimitiveSchema.LONG:
                return struct.unpack_from('!q', read(8))[0]
            elif schema == PrimitiveSchema.DOUBLE:
                return struct.unpack_from('!d', read(8))[0]
            elif schema == PrimitiveSchema.STRING:
                return read_string()
            elif schema == PrimitiveSchema.BOOLEAN:
                return struct.unpack_from('!?', read(1))[0]
            else:
                raise ValueError(f'Unknown primitive schema: {schema}')
        elif isinstance(schema, EnumSchema):
            ordinal, = struct.unpack_from('!i', read(4))
            return schema.constants[ordinal]
        else:
            raise ValueError(f'Unknown schema: {schema}')


    magic, version = struct.unpack_from('!2sh', read(4))
    assert magic == b'RR'
    assert version == 0

    channels = []
    schemas = {}
    messages = defaultdict(list)

    while True:
        try:
            entry_type, = struct.unpack_from('!i', read(4))
            if entry_type == 0:
                # channel definition
                ch = read_string()
                schemas[ch] = read_schema()
                channels.append(ch)
            elif entry_type == 1:
                # message 
                ch_index, = struct.unpack_from('!i', read(4))
                ch = channels[ch_index]
                messages[ch].append(read_msg(schemas[ch]))
            else:
                raise ValueError(f"Unknown entry type: {entry_type}")

        except EOFError:
            break

    return schemas, dict(messages)


def log_data(schemas, messages, timestamp_keys=('timestamp', 'time')):
    # Lists and dictionaries for categorized data
    config = {}
    timeline_events = defaultdict(list)
    timeline = defaultdict(dict)
    line_graphs = defaultdict(list)
    
    # Process each channel based on the schema and message structure
    for ch, schema in schemas.items():
        # Retrieve messages for this channel
        ch_messages = messages[ch]
        
        # Store struct messages without timestamp keys in config
        if isinstance(schema, StructSchema):
            # Check if the struct contains any timestamp-like fields
            has_timestamp_field = any(
                key.lower() in timestamp_keys for key in schema.fields.keys()
            )
            if not has_timestamp_field:
                config[ch] = ch_messages[0]
            else:
                # Extract timestamp and other fields, store in timeline
                for msg in ch_messages:
                    timestamp_key = next(
                        key for key in schema.fields if key.lower() in timestamp_keys
                    )
                    timestamp = msg[timestamp_key]
                    # Add other fields (excluding the timestamp field) to the timeline entry
                    timeline[ch][timestamp] = {
                        key: value for key, value in msg.items() if key != timestamp_key
                    }

        # Store series data for channels with multiple numerical or timestamp entries
        elif isinstance(schema, PrimitiveSchema) and schema == PrimitiveSchema.DOUBLE and len(ch_messages) > 1:
            line_graphs[ch] = ch_messages

        elif isinstance(schema, PrimitiveSchema) and schema == PrimitiveSchema.LONG and len(ch_messages) > 0:
            for event in ch_messages:
                timeline_events[event].append(ch)

    return config, timeline, line_graphs, timeline_events

def plot_timeline(timeline, timeline_events):
    # Plot each channel's data that contains timestamped entries
    for ch, entries in timeline.items():
        plt.figure()
        timestamps = sorted(entries.keys())
        # Plot each data field over time
        for field in entries[timestamps[0]].keys():
            values = [entries[t][field] for t in timestamps]
            i = 0
            for timestamp, labels in timeline_events.items():
                # HORRIBLE BAD BAD O(n), Could be O(log n) since it's sorted but eh
                plt.annotate(", ".join(labels), (timestamp, entries[min(timestamps, key=lambda x:abs(x-timestamp))][field]), 
                            xytext=(2, 0), textcoords="offset fontsize",
                            arrowprops=dict(arrowstyle='-|>'))
                i+=1

            plt.plot(timestamps, values, label=field)
        plt.title(f"Channel: {ch} (Timestamped Data)")
        plt.xlabel("Timestamp")
        plt.ylabel("Value")
        plt.legend()
        plt.grid()
        # plt.show()

def plot_line_graphs(line_graphs):
    for ch, data in line_graphs.items():
        plt.figure()
        plt.plot(range(len(data)), data, marker='o')
        plt.title(f"Channel: {ch}")
        plt.xlabel("Sample Number")
        plt.ylabel("Value")
        plt.grid()
        # plt.show()

def download_log_file(logs_url):
    # Fetch the logs page
    response = requests.get(logs_url)
    if response.status_code != 200:
        raise Exception(f"Failed to load logs page: {response.status_code}")
    
    # Parse HTML to find the first log link
    soup = BeautifulSoup(response.text, 'html.parser')
    first_link = soup.find('a', href=True)  # Find the first link
    
    # Check if link is valid
    if not first_link or 'download?file=' not in first_link['href']:
        raise Exception("No valid log file link found on the logs page.")

    # Construct the full URL for the download
    log_file_url = f"http://192.168.43.1:8080{first_link['href']}"
    
    # Download the log file
    log_response = requests.get(log_file_url)
    if log_response.status_code != 200:
        raise Exception(f"Failed to download log file: {log_response.status_code}")

    # Return the log file as a byte stream for further processing
    return io.BytesIO(log_response.content)

if __name__ == '__main__':
    import sys

    if sys.argv[1] == 'latest':
        logs_url = "http://192.168.43.1:8080/logs"

        # Download and parse the log file
        log_file = download_log_file(logs_url)

        schemas, messages = read_file(log_file)
    else:
        filepath = sys.argv[1]
    
        with open(filepath, 'rb') as f:
            schemas, messages = read_file(f)

    filterchannels = sys.argv[2:] if len(sys.argv) > 2 else [ch for ch in schemas.keys()]
    block_channels = ["FINISHED_FINISHED","THREE_DEAD_WHEEL_INPUTS"]
    channel_names = set()

    for name in schemas.keys():
        for key in filterchannels:
            if key in name:
                blocked = len([block for block in block_channels if block in name]) > 0
                if not blocked:
                    channel_names.add(name)

    print(channel_names)

    schemas = dict((k, v) for k, v in schemas.items() if k in channel_names)
    messages = dict((k, v) for k, v in messages.items() if k in channel_names)
    
    for ch, schema in schemas.items():
        print(f'Channel: {ch} ({len(messages[ch])} messages)\n  {schema}')
    
    config, timeline, line_graphs, timeline_events = log_data(schemas, messages, timestamp_keys=('timestamp', 'time'))

    # Print config data
    print("Config Data:", config)

    # Plot timeline and line graphs
    plt.ion()
    plot_timeline(timeline, timeline_events)
    plot_line_graphs(line_graphs)
    plt.show(block=True)
