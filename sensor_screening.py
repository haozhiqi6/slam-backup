"""Small non-retiming filters for this setup's little-endian Livox CDR."""
import struct
import numpy as np


def header_ns(data):
    if bytes(data[:2])!=bytes([0,1]):raise ValueError('Expected little-endian XCDR1')
    sec,ns=struct.unpack_from('<iI',data,4)
    return sec*10**9+ns


def screen_imu(data,previous_ns=None):
    head=header_ns(data)
    info={'header_ns':head,'header_step_s':None if previous_ns is None else (head-previous_ns)*1e-9}
    if previous_ns is not None and head<=previous_ns:
        return None,{**info,'reason':'imu_out_of_order'}
    return data,info


def screen_lidar(data,previous_ns=None,reference_imu_ns=None):
    head=header_ns(data)
    info={'header_ns':head,'header_step_s':None if previous_ns is None else (head-previous_ns)*1e-9,
          'dropped_points':0,'reordered':False}
    if previous_ns is not None and head<=previous_ns:
        return None,{**info,'reason':'lidar_out_of_order'}
    # This setup normally publishes LiDAR headers about 0.1 s ahead of the
    # latest IMU. Quarantine a far-future frame without advancing the accepted
    # header, so one transient clock label cannot poison following good frames.
    # No timestamp is rewritten; sustained lack of usable data is handled by
    # the capture watchdog. The bound is specific to this 10 Hz setup.
    if reference_imu_ns is not None and head-reference_imu_ns>350_000_000:
        return None,{**info,'reason':'lidar_header_far_ahead_of_imu',
                     'header_ahead_of_imu_s':(head-reference_imu_ns)*1e-9}
    length=struct.unpack_from('<I',data,12)[0]
    offset=4+((16+length-4+7)//8)*8
    n=struct.unpack_from('<I',data,offset+16)[0]
    if n!=struct.unpack_from('<I',data,offset+8)[0]:raise ValueError('Point counts disagree')
    start=offset+20
    # The final record has 19 serialized bytes, without trailing alignment.
    times=np.ndarray((n,),dtype='<u4',buffer=data,offset=start,strides=(20,))
    keep=times<=150_000_000
    info['dropped_points']=int((~keep).sum())
    kept_times=times[keep]
    if not len(kept_times):return None,{**info,'reason':'no_points_with_usable_time'}
    order=np.argsort(kept_times,kind='stable') if (np.diff(kept_times.astype(np.int64))<0).any() else None
    info['reordered']=order is not None
    if info['dropped_points']==0 and order is None:return data,info
    padded=data+bytes(max(0,start+n*20-len(data)))
    records=np.ndarray((n,),dtype='V20',buffer=padded,offset=start)[keep]
    if order is not None:records=records[order]
    prefix=bytearray(data[:start]);struct.pack_into('<I',prefix,offset+8,len(records));struct.pack_into('<I',prefix,offset+16,len(records))
    return bytes(prefix)+records.tobytes(),info
