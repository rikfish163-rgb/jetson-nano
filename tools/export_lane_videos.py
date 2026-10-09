#!/usr/bin/env python3
"""Offline export of timestamp-matched front/BEV frames and control captions."""
import argparse
import bisect
import json
import re
import shutil
import subprocess
from pathlib import Path


def images(folder, key):
    pattern = re.compile(r'^(\d+)_' + key + r'\.jpg$')
    return {int(match.group(1)): p.resolve() for p in folder.glob('*_' + key + '.jpg')
            for match in [pattern.match(p.name)] if match}


def concat_text(stamps, frames):
    rows = ['ffconcat version 1.0\n']
    intervals = [(b-a)/1e9 for a,b in zip(stamps,stamps[1:])]
    last = sorted(intervals)[len(intervals)//2] if intervals else .1
    for index, stamp in enumerate(stamps):
        filename = str(frames[stamp]).replace("'", "'\\''")
        rows.append("file '%s'\n" % filename)
        duration = intervals[index] if index < len(intervals) else last
        rows.append('duration %.9f\n' % duration)
    rows.append("file '%s'\n" % str(frames[stamps[-1]]).replace("'", "'\\''"))
    return ''.join(rows),last


def srt_time(seconds):
    millis = max(0,round(seconds*1000))
    return '%02d:%02d:%02d,%03d' % (millis//3600000,millis//60000%60,
                                     millis//1000%60,millis%1000)


def control_rows(folder):
    rows = []
    for filename in ('status.jsonl','target.jsonl'):
        path = folder/filename
        if not path.exists():
            continue
        for line in path.read_text().splitlines():
            data = json.loads(line)['data']
            command = data.get('command',{})
            if not isinstance(command,dict):
                command = data.get('right_turn_debug',{}).get('encoded_command',{})
            task = data.get('timed_bypass') or {}
            rows.append((float(data['stamp']),data.get('state','?'),task.get('phase',''),
                         command.get('speed_raw','?'),command.get('steering_raw','?'),
                         data.get('reason','')))
    return sorted(rows,key=lambda row:row[0])


def export(folder, output):
    if not shutil.which('ffmpeg'):
        raise RuntimeError('ffmpeg is required for offline export')
    front,bev = images(folder,'front'),images(folder,'bev')
    stamps = sorted(set(front)&set(bev))
    if not stamps:
        raise RuntimeError('No front/BEV pairs with identical source timestamps')
    summary_path = folder/'summary.json'
    summary = json.loads(summary_path.read_text()) if summary_path.exists() else {}
    if summary.get('closed') is False and not summary.get('stopped_reason'):
        raise RuntimeError('Recorder is still active; stop it before exporting')
    output.mkdir(parents=True,exist_ok=True)
    output = output.resolve()
    controls = control_rows(folder)
    control_stamps = [row[0] for row in controls]
    for key,frames in (('front',front),('bev',bev)):
        content,last_duration = concat_text(stamps,frames)
        (output/(key+'.ffconcat')).write_text(content)
    captions = []
    start = stamps[0]/1e9
    for index,stamp in enumerate(stamps):
        current = stamp/1e9
        end = stamps[index+1]/1e9 if index+1 < len(stamps) else current+last_duration
        at = bisect.bisect_right(control_stamps,current)-1
        text = 't+%.3fs  camera stamp %.6f' % (current-start,current)
        if at >= 0:
            row = controls[at]
            text += '\n%s %s  speed=%s steer=%s  %s' % row[1:]
            if current-row[0]>.5:
                text += '  [control data older than 0.5s]'
        captions.append('%d\n%s --> %s\n%s\n\n' %
            (index+1,srt_time(current-start),srt_time(end-start),text))
    (output/'controls.srt').write_text(''.join(captions))
    common = ['ffmpeg','-hide_banner','-loglevel','error','-y']
    codec = ['-vsync','vfr','-c:v','libx264','-preset','veryfast','-crf','23',
             '-pix_fmt','yuv420p','-movflags','+faststart']
    for key in ('front','bev'):
        subprocess.run(common+['-f','concat','-safe','0','-i',key+'.ffconcat',
            '-vf','pad=ceil(iw/2)*2:ceil(ih/2)*2']+codec+[key+'.mp4'],cwd=str(output),check=True)
    graph = ('[0:v]scale=640:360,pad=640:400:0:20[a];'
             '[1:v]scale=480:400,pad=640:400:80:0[b];'
             '[a][b]hstack=inputs=2,subtitles=controls.srt[v]')
    subprocess.run(common+['-i','front.mp4','-i','bev.mp4','-filter_complex',graph,
        '-map','[v]']+codec+['comparison.mp4'],cwd=str(output),check=True)
    manifest = dict(recording=str(folder.resolve()),paired_frames=len(stamps),
        first_source_stamp_ns=stamps[0],last_source_stamp_ns=stamps[-1],
        unpaired_front=len(set(front)-set(bev)),unpaired_bev=len(set(bev)-set(front)),
        gaps_over_half_second=sum(b-a>500000000 for a,b in zip(stamps,stamps[1:])),
        timing=('Source-stamp intervals supplied to both videos identically; the JPEG '
                'demuxer rounds playback timestamps to 40 ms. Gaps hold the preceding '
                'recorded frame; exact source timestamps remain in filenames and logs.'),
        view='BEV is a camera transform, not an external overhead view of the vehicle.',
        recorder_summary=summary)
    (output/'manifest.json').write_text(json.dumps(manifest,indent=2))
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording',type=Path)
    parser.add_argument('--latest',action='store_true',help='Select newest run within a recording root')
    parser.add_argument('--output',type=Path)
    args = parser.parse_args()
    folder = args.recording
    if args.latest:
        runs = sorted(p for p in folder.iterdir() if p.is_dir() and (p/'summary.json').exists())
        if not runs:
            parser.error('No recording directories found')
        folder = runs[-1]
    try:
        manifest = export(folder,args.output or folder/'videos')
    except (RuntimeError,ValueError,OSError,subprocess.CalledProcessError) as exc:
        parser.exit(1,str(exc)+'\n')
    print(json.dumps(manifest,indent=2))


if __name__=='__main__':
    main()
