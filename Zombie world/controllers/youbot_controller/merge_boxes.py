# merge bounding boxes that overlap
def merge_boxes(boxes):
    unused_boxes = boxes.copy()
    used_boxes = []
    merged_boxes = []
    for idx, box in enumerate(boxes):
        x, y, w, h = box
        for box2 in boxes:
            x2, y2, w2, h2 = box2
            if box != box2 and box2 not in used_boxes:
                if x2 < x + w and x2 + w2 > x and y2 < y + h and y2 + h2 > y:
                    x = min(x, x2)
                    y = min(y, y2)
                    w = max(x + w, x2 + w2) - x
                    h = max(y + h, y2 + h2) - y
                    # print('merged box', (x, y, w, h), 'from', box, box2)
                    merged_boxes.append((x, y, w, h))
                    used_boxes.append(box)
                    if (box in unused_boxes):
                        unused_boxes.remove(box)
                    if (box2 in unused_boxes):
                        unused_boxes.remove(box2)
    for box in unused_boxes:
        merged_boxes.append(box)
    return merged_boxes