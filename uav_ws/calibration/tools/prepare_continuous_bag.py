"""Create a separate continuous Kalibr input; preserve stereo pairs and all IMU."""
from collections import Counter
import json
from pathlib import Path
import shutil

from rosbags.rosbag1 import Reader, Writer
from rosbags.typesys import Stores, get_typestore

WS=Path('/home/orangepi/BoomBoomFly/uav_ws')
SOURCE=WS/'calibration/vio_calib_20261018_112654_iZIADC'
OUT=WS/'calibration/vio_calib_run_20261018'
START_S=20.
END_S=144.
IMAGE_PERIOD_NS=250_000_000  # 4 Hz, selected using matching source stamps
store=get_typestore(Stores.ROS1_NOETIC)

def stamp(message):
    return message.header.stamp.sec*10**9+message.header.stamp.nanosec

with Reader(SOURCE/'calibration_full.bag') as reader:
    start=reader.start_time
    lo=start+int(START_S*1e9)
    hi=start+int(END_S*1e9)
    image_stamps={}
    for connection in reader.connections:
        if connection.msgtype!='sensor_msgs/msg/Image':continue
        values=[]
        for _,ts,data in reader.messages(connections=[connection],start=lo,stop=hi):
            values.append(stamp(store.deserialize_ros1(data,connection.msgtype)))
        image_stamps[connection.topic]=set(values)
    pairs=sorted(set.intersection(*image_stamps.values()))
    selected=[]
    for value in pairs:
        if not selected or value-selected[-1]>=IMAGE_PERIOD_NS:selected.append(value)
    chosen=set(selected)
    counts=Counter()
    times={}
    target=OUT/'continuous_20_144_4hz.bag'
    with Writer(target) as writer:
        mapping={c.id:writer.add_connection(c.topic,c.msgtype,msgdef=c.msgdef.data,
                  md5sum=c.digest,callerid=c.ext.callerid,latching=c.ext.latching)
                 for c in reader.connections}
        for c,ts,data in reader.messages(start=lo,stop=hi):
            m=store.deserialize_ros1(data,c.msgtype)
            s=stamp(m)
            if c.msgtype=='sensor_msgs/msg/Image' and s not in chosen:continue
            writer.write(mapping[c.id],ts,data)
            counts[c.topic]+=1
            times.setdefault(c.topic,[]).append(s)
    report={'source':str(SOURCE/'calibration_full.bag'),'output':str(target),
            'interval_relative_to_record_start_s':[START_S,END_S],
            'image_rate_hz':4,'paired_image_stamps':len(selected),'topics':{}}
    for n,v in times.items():
        gaps=[(b-a)/1e9 for a,b in zip(v,v[1:])]
        report['topics'][n]={'count':counts[n],'source_max_gap_s':max(gaps),
                             'backwards':sum(g<0 for g in gaps),'duplicates':sum(g==0 for g in gaps)}
    (OUT/'prepared_input.json').write_text(json.dumps(report,indent=2))
    shutil.copyfile(SOURCE/'target.yaml',OUT/'target.yaml')
    print(json.dumps(report,indent=2))
