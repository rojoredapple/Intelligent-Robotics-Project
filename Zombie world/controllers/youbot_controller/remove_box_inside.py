def remove_box_inside(boxes):
    for idx, box in enumerate(boxes):
        x, y, w, h = box
        for box2 in boxes:
            x2, y2, w2, h2 = box2
            if box != box2:
                if x2 < x + w and x2 + w2 > x and y2 < y + h and y2 + h2 > y:
                    boxes.remove(box2)
    return boxes