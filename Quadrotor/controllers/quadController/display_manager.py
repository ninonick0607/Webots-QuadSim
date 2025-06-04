def clamp(value, low, high):
    return max(low, min(value, high))

def draw_axes(display_dev, x, y, width, height, min_val, max_val, tick_count=5):
    display_dev.setColor(0x404040)
    display_dev.drawRectangle(x, y, width, height)
    
    display_dev.setColor(0xFFFFFF)
    display_dev.drawLine(x, y, x, y + height - 1)
    display_dev.drawLine(x, y + height - 1, x + width - 1, y + height - 1)
    
    display_dev.setFont("Arial", 14, True)
    for i in range(tick_count + 1):
        frac = i / tick_count
        val = min_val + (max_val - min_val) * (1 - frac)
        yy = y + int(frac * (height - 1))
        display_dev.drawLine(x - 5, yy, x, yy)
        txt = f"{val:.2f}"
        display_dev.setColor(0xFFFF00)
        display_dev.drawText(txt, x + 3, yy - 7)
    
    display_dev.setFont("Arial", 12, True)
    for i in range(tick_count + 1):
        xx = x + int(i * (width - 1) / tick_count)
        display_dev.setColor(0xFFFFFF)
        display_dev.drawLine(xx, y + height - 1, xx, y + height - 1 - 5)
        label = str(xx - x)
        display_dev.drawText(label, xx - (len(label)*6)//2, y + height + 2)

def draw_series(display_dev, history, x, y, width, height, color, min_val, max_val):
    n = len(history)
    if n < 2:
        return
    
    pts = list(history)
    maxlen = history.maxlen
    
    for i in range(n - 1):
        xx1 = x + int(i * (width - 1) / (maxlen - 1))
        yy1_val = pts[i]
        yy1 = y + int((max_val - yy1_val) * (height - 1) / (max_val - min_val))
        yy1 = clamp(yy1, y, y + height - 1)
        
        xx2 = x + int((i + 1) * (width - 1) / (maxlen - 1))
        yy2_val = pts[i + 1]
        yy2 = y + int((max_val - yy2_val) * (height - 1) / (max_val - min_val))
        yy2 = clamp(yy2, y, y + height - 1)
        
        display_dev.setColor(color)
        if xx1 == xx2 and yy1 == yy2:
            display_dev.drawPixel(xx1, yy1)
        else:
            display_dev.drawLine(xx1, yy1, xx2, yy2)

def draw_two_series_with_axes(display_dev, hist1, hist2, x, y, width, height,
                              color1, color2, min_val, max_val, label, tick_count=5):
    draw_axes(display_dev, x, y, width, height, min_val, max_val, tick_count)
    
    display_dev.setColor(0xFFFFFF)
    display_dev.setFont("Arial", 12, True)
    display_dev.drawText(label, x + 5, y + 15)
    
    draw_series(display_dev, hist1, x, y, width, height, color1, min_val, max_val)
    draw_series(display_dev, hist2, x, y, width, height, color2, min_val, max_val)

def calculate_bounds(histories):
    all_values = []
    for hist in histories:
        all_values.extend(list(hist))
    
    if not all_values:
        return 0.0, 1.0
    
    min_val = min(all_values)
    max_val = max(all_values)
    
    if abs(max_val - min_val) < 1e-6:
        max_val = min_val + 1e-3
    
    return min_val, max_val

def render_all_plots(display_dev, display_width, display_height,
                     altitude_desired_hist, altitude_actual_hist,
                     roll_desired_hist, roll_actual_hist,
                     pitch_desired_hist, pitch_actual_hist,
                     yaw_desired_hist, yaw_actual_hist):
    
    graph_count = 4
    plot_height_per_graph = display_height // graph_count
    
    display_dev.setColor(0x000000)
    display_dev.fillRectangle(0, 0, display_width, display_height)
    
    plot_configs = [
        {
            'desired_hist': altitude_desired_hist,
            'actual_hist': altitude_actual_hist,
            'y_offset': 0,
            'desired_color': 0xFFFF00,
            'actual_color': 0x00FF00,
            'label': "Altitude (m)"
        },
        {
            'desired_hist': roll_desired_hist,
            'actual_hist': roll_actual_hist,
            'y_offset': plot_height_per_graph,
            'desired_color': 0xFFFF00,
            'actual_color': 0xFF0000,
            'label': "Roll (rad)"
        },
        {
            'desired_hist': pitch_desired_hist,
            'actual_hist': pitch_actual_hist,
            'y_offset': 2 * plot_height_per_graph,
            'desired_color': 0xFFFF00,
            'actual_color': 0x0000FF,
            'label': "Pitch (rad)"
        },
        {
            'desired_hist': yaw_desired_hist,
            'actual_hist': yaw_actual_hist,
            'y_offset': 3 * plot_height_per_graph,
            'desired_color': 0xFFFF00,
            'actual_color': 0xFF00FF,
            'label': "Yaw Rate (rad/s)"
        }
    ]
    
    for config in plot_configs:
        min_val, max_val = calculate_bounds([config['desired_hist'], config['actual_hist']])
        draw_two_series_with_axes(
            display_dev, 
            config['desired_hist'], 
            config['actual_hist'],
            0, 
            config['y_offset'], 
            display_width, 
            plot_height_per_graph,
            config['desired_color'], 
            config['actual_color'], 
            min_val, 
            max_val, 
            config['label'], 
            tick_count=5
        )