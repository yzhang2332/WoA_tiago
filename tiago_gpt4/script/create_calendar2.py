import tkinter as tk
from tkinter import font as tkfont
from datetime import datetime, timedelta
import pytz
import os.path
import pickle
from google.oauth2.credentials import Credentials
from google_auth_oauthlib.flow import InstalledAppFlow
from google.auth.transport.requests import Request
from googleapiclient.discovery import build
from datetime import timezone
from Showing_Events_Caleder import Showing_Events_Calender


class EventCalendarCreator:
    def __init__(self):

        self.event_data = None
        self.window = None
        self.create_event_calendar()


    

    def create_event_calendar(self):
        # Function to round up time to the nearest hour
        def round_up_time(dt):
            melbourne_tz = pytz.timezone('Australia/Melbourne')
            dt_min = datetime.min.replace(tzinfo=timezone.utc).astimezone(melbourne_tz)
            return dt + (dt_min - dt) % timedelta(hours=1)
        

        def convert_time(time_str):
            # Convert the time string to a datetime object
            time_obj = datetime.strptime(time_str, '%H:%M')
            
            # Format the time object to 12-hour format with AM/PM
            time_12_hour = time_obj.strftime('%I:%M %p')
            
            # Split the hour and minute
            hour = int(time_obj.strftime('%I'))
        
            minute = int(time_obj.strftime('%M'))
            am_pm = time_obj.strftime('%p')

            if float(minute)>0:
                if minute<10:
                    new_time = f"{str(hour)} O {minute} {am_pm}"
                else:
                    new_time = f"{str(hour)} {minute} {am_pm}"
            else:
                new_time = f"{str(hour)} {am_pm}"
            
            return new_time 

        # Function to create event
        def create_event():
            attendees_emails = attendees_entry.get().split(';')  # Assuming emails are separated by semicolons
            attendees_list = [{'email': email.strip()} for email in attendees_emails if email.strip()]

            event = {
                'summary': title_entry.get(),
                'location': location_entry.get(),
                'description': "",
                'start': {
                    'dateTime': start_date_entry.get() + 'T' + start_time_entry.get() + ':00',
                    'timeZone': 'Australia/Melbourne',
                },
                'end': {
                    'dateTime': end_date_entry.get() + 'T' + end_time_entry.get() + ':00',
                    'timeZone': 'Australia/Melbourne',
                },
                'attendees': attendees_list,
                'reminders': {
                    'useDefault': False,
                    'overrides': [
                        {'method': 'email', 'minutes': 24 * 60},
                        {'method': 'popup', 'minutes': 10},
                    ],
                },
            }
        
            created_event = service.events().insert(calendarId='primary', body=event).execute()
            print('Event created: %s' % (created_event.get('htmlLink')))

            if 'summary' in event:
                Title = event['summary']
            else:
                Title = "without title"

            # Extracting start and end times from the provided data
            start_time = event['start']['dateTime']
            end_time = event['end']['dateTime']
            
            # Formatting the times
            start_time_formatted = start_time.split('T')[1][:5]
            end_time_formatted = end_time.split('T')[1][:5]


            start_time_formatted_new = convert_time(start_time_formatted)
            end_time_formatted_new = convert_time(end_time_formatted)




            
            formatted_time_range = f"From {start_time_formatted_new} to {end_time_formatted_new}"

            if 'location' in event:
                Location = event['location']
            else:
                Location = "a not specified location"

            if 'attendees' in event:
                attendees = event.get('attendees', [])
                attendee_emails = [attendee['email'] for attendee in attendees]
            else:
                attendee_emails = "No attendees"

            self.event_data = "Your meeting " + Title + " " + formatted_time_range + " at " + Location + " has been booked successfully."

            self.window.after(1000, self.window.destroy())

        # Google Calendar API setup
        SCOPES = ['https://www.googleapis.com/auth/calendar']
        creds = None
        if os.path.exists('token.pickle'):
            with open('token.pickle', 'rb') as token:
                creds = pickle.load(token)
        if not creds or not creds.valid:
            if creds and creds.expired and creds.refresh_token:
                creds.refresh(Request())
            else:
                flow = InstalledAppFlow.from_client_secrets_file('/home/pal/tiago_ws/src/woa_tiago/tiago_gpt4/script/credentials1.json', SCOPES)
                creds = flow.run_local_server(port=0)
            with open('token.pickle', 'wb') as token:
                pickle.dump(creds, token)
        service = build('calendar', 'v3', credentials=creds)




        # Tkinter window setup
        self.window = tk.Tk()
        self.window.title("Google Calendar Event Creator")

        # Make the window full screen
        self.window.attributes('-fullscreen', True)

        # Set a larger font size for the widgets
        large_font = tkfont.Font(family="Helvetica", size=30)

        # Create a frame to hold the widgets, and place it in the center
        center_frame = tk.Frame(self.window)
        center_frame.place(relx=0.5, rely=0.5, anchor=tk.CENTER)

        # Increase the width of the entry fields
        entry_width = 40

        tk.Label(center_frame, text="Title", font=large_font).grid(row=0, column=0)
        title_entry = tk.Entry(center_frame, font=large_font, width=entry_width)
        title_entry.grid(row=0, column=1)

        # Attendee field with default value
        tk.Label(center_frame, text="Attendees (email;email)", font=large_font).grid(row=1, column=0)
        attendees_entry = tk.Entry(center_frame, font=large_font, width=entry_width)
        attendees_entry.insert(0, "stabatabaeim@student.unimelb.edu.au")
        attendees_entry.grid(row=1, column=1)

        tk.Label(center_frame, text="Location", font=large_font).grid(row=2, column=0)
        location_entry = tk.Entry(center_frame, font=large_font, width=entry_width)
        location_entry.grid(row=2, column=1)

        # Pre-fill current date and time in Melbourne timezone
        melbourne_tz = pytz.timezone('Australia/Melbourne')
        now = datetime.now(melbourne_tz)
        rounded_now = round_up_time(now)
        one_hour_later = rounded_now + timedelta(hours=1)

        tk.Label(center_frame, text="Start Date (YYYY-MM-DD)", font=large_font).grid(row=3, column=0)
        start_date_entry = tk.Entry(center_frame, font=large_font, width=entry_width)
        start_date_entry.insert(0, rounded_now.strftime('%Y-%m-%d'))
        start_date_entry.grid(row=3, column=1)

        tk.Label(center_frame, text="Start Time (HH:MM)", font=large_font).grid(row=4, column=0)
        start_time_entry = tk.Entry(center_frame, font=large_font, width=entry_width)
        start_time_entry.insert(0, rounded_now.strftime('%H:%M'))
        start_time_entry.grid(row=4, column=1)

        tk.Label(center_frame, text="End Date (YYYY-MM-DD)", font=large_font).grid(row=5, column=0)
        end_date_entry = tk.Entry(center_frame, font=large_font, width=entry_width)
        end_date_entry.insert(0, one_hour_later.strftime('%Y-%m-%d'))
        end_date_entry.grid(row=5, column=1)

        tk.Label(center_frame, text="End Time (HH:MM)", font=large_font).grid(row=6, column=0)
        end_time_entry = tk.Entry(center_frame, font=large_font, width=entry_width)
        end_time_entry.insert(0, one_hour_later.strftime('%H:%M'))
        end_time_entry.grid(row=6, column=1)

        submit_button = tk.Button(center_frame, text="Create Event", command=create_event, font=large_font)
        submit_button.grid(row=7, column=0, columnspan=2)


        self.window.mainloop()



    def get_event_data(self):
        return self.event_data

if __name__ == '__main__':
    event_creator = EventCalendarCreator()
    event_data = event_creator.get_event_data()
    print(event_data)
