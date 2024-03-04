#!/usr/bin/env python

from flask import Flask, send_file
import threading

app = Flask(__name__)

# initial flag
flag = False

@app.route('/')
def show_image():
    global flag
    if flag:
        return send_file('../image/listening.gif', mimetype='image/gif')
    else:
        return send_file('../image/speaking.gif', mimetype='image/gif')

def set_signal_flag(new_value):
    # Update the flag with a new value
    global flag
    flag = new_value

def create_and_run_app():
    app.run(host='0.0.0.0', port=5000, debug=True, use_reloader=False)

def run_app_in_thread():
    thread = threading.Thread(target=create_and_run_app)
    thread.daemon = True
    thread.start()

# Export the set_flag function so it can be used from other files
__all__ = ['run_app_in_thread', 'set_signal_flag']


if __name__ == '__main__':
    run_app_in_thread()
    set_signal_flag(False)
    # app = create_flask_app(True)
    # app.run(host='0.0.0.0', port=5000, debug=True, use_reloader=False)  # Prevent Flask from starting the app twice