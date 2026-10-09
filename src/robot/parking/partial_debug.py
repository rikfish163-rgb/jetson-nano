"""Draw partial-model evidence on the existing stationary debug image."""
from __future__ import division

import cv2
import numpy as np


def draw_partial_diagnostics(image, diagnostic, metric):
    if diagnostic.get('algorithm') != 'partial_model_v1':
        return
    model = diagnostic.get('model', {})
    origin = np.asarray(metric(0, 0), dtype=float)
    basis = np.column_stack((np.asarray(metric(1, 0)) - origin,
                             np.asarray(metric(0, 1)) - origin))
    inverse = np.linalg.inv(basis)

    def pixel(point):
        return np.rint(np.dot(inverse, np.asarray(point) - origin)).astype(int)

    # These outlines are hypotheses, not obstacle-free parking commands.
    for index, hypothesis in enumerate(model.get('hypotheses', [])[:3]):
        points = [pixel(p) for p in hypothesis.get('polygon', [])]
        if len(points) != 4:
            continue
        for a, b in zip(points, points[1:] + points[:1]):
            length = float(np.linalg.norm(b - a))
            if not length:
                continue
            for start in range(0, int(length), 18):
                p = a + (b - a) * (start / length)
                q = a + (b - a) * (min(start + 9, length) / length)
                cv2.line(image, tuple(p.astype(int)), tuple(q.astype(int)),
                         (160, 160, 160), 2)
        anchor = points[0].copy()
        anchor[0] = np.clip(anchor[0], 5, image.shape[1] - 280)
        anchor[1] = np.clip(anchor[1] - 8, 24, image.shape[0] - 160)
        cv2.putText(image, 'H%d score=%.2f' % (
            index + 1, hypothesis.get('score', 0)), tuple(anchor),
            cv2.FONT_HERSHEY_SIMPLEX, .7, (200, 200, 200), 2)

    for guide in model.get('entrance_guides', []):
        for point in guide.get('dash_centres', []):
            cv2.circle(image, tuple(pixel(point)), 7, (255, 255, 0), 2)

    for part in model.get('parts', {}).values():
        if part.get('supported'):
            for a, b in part.get('segments', []):
                cv2.line(image, tuple(pixel(a)), tuple(pixel(b)), (255, 220, 60), 3)

    panel_top = max(0, image.shape[0] - 150)
    panel = image.copy()
    cv2.rectangle(panel, (0, panel_top),
                  (image.shape[1] - 1, image.shape[0] - 1), (12, 20, 25), -1)
    cv2.addWeighted(panel, .85, image, .15, 0, image)
    parts = model.get('parts', {})
    side_labels = []
    for i in range(3):
        part = parts.get('side_%d' % i, {})
        side_labels.append('S%d %s %.0fcm' % (
            i, 'YES' if part.get('supported') else '?',
            100 * part.get('matched_length_m', 0)))
    back = parts.get('back', {})
    lines = [
        'PARTIAL MODEL v2 | %d supported bays | %s' % (
            diagnostic.get('accepted', 0),
            'NEAR/FAR AMBIGUOUS' if model.get('ambiguous') else
            model.get('reason', 'waiting')),
        'Score %.2f (not probability) | visible %.0f%% | %d poses | %.0f ms' % (
            model.get('score', 0), 100 * model.get('visible_fraction', 0),
            model.get('evaluated', 0), model.get('processing_ms', 0)),
        '%s | back %s | dashes %d | mouth anchors %d' % (
            ' / '.join(side_labels), 'YES' if back.get('supported') else '?',
            model.get('dash_count', 0), model.get('mouth_anchors', 0)),
        '%d entrance guides | cyan circles = entrance dashes | gray = hypotheses' %
        len(model.get('entrance_guides', [])),
    ]
    for index, line in enumerate(lines):
        cv2.putText(image, line, (12, panel_top + 28 + 33 * index),
                    cv2.FONT_HERSHEY_SIMPLEX, .70, (225, 225, 225), 2)
