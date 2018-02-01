#!/usr/bin/env python2
from __future__ import print_function
import requests
import sys

# Download the files through shared google drive links
def download_file_from_google_drive(id, destination):
    def get_confirm_token(response):
        for key, value in response.cookies.items():
            if key.startswith('download_warning'):
                return value

        return None

    def save_response_content(response, destination):
        CHUNK_SIZE = 32768

        with open(destination, "wb") as f:
            for chunk in response.iter_content(CHUNK_SIZE):
                if chunk: # filter out keep-alive new chunks
                    f.write(chunk)

    URL = "https://docs.google.com/uc?export=download"

    session = requests.Session()

    response = session.get(URL, params = { 'id' : id }, stream = True)
    token = get_confirm_token(response)

    if token:
        params = { 'id' : id, 'confirm' : token }
        response = session.get(URL, params = params, stream = True)

    save_response_content(response, destination)    


if __name__ == "__main__":
    try:

        # Download 'prototext'
        print("Downloading the 'prototxt' file...", end='   ')
        download_file_from_google_drive('1zihnef4G2jKwqffFapaP6gUIg18l3pf6', './reground.prototxt')
        print("[done]")

        # Download 'words'
        print("Downloading the 'words' file...", end='   ')
        download_file_from_google_drive('1FvWF-1SK6A4GDLdwzOvwhBO_DSuirFzH', './reground_words.txt')
        print("[done]")

        # Download the acual 'model'
        print("Downloading the 'model' file...", end='   ')
        download_file_from_google_drive('1_r8tlD7o0EEcb1_3SYkblqqyiCc8nfZN', './reground_googlenet.caffemodel')
        print("[done]")
        
    except:
        print("Could not download the model... ")
    
