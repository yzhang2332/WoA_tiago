from datetime import datetime

def convert_time(time_str):
    # Convert the time string to a datetime object
    time_obj = datetime.strptime(time_str, '%H:%M')
    
    # Format the time object to 12-hour format with AM/PM
    time_12_hour = time_obj.strftime('%I:%M %p')
    
    # Split the hour and minute
    hour = time_obj.strftime('%I')
    minute = time_obj.strftime('%M')
    am_pm = time_obj.strftime('%p')

    if float(minute)>0:
        print(f"{hour} {minute} {am_pm}")
    else:
        print(f"{hour} {am_pm}")
    
    return time_12_hour, hour, minute, am_pm

time_str = "20:00"
time_12_hour, hour, minute, am_pm = convert_time(time_str)

# print(f"12-hour format: {time_12_hour}")
# print(f"Hour: {hour}")
# print(f"Minute: {minute}")
# print(f"AM/PM: {am_pm}")

if float(minute)>0:
    print(f"{hour} {minute} {am_pm}")
else:
    print(f"{hour} {am_pm}")

