from datetime import datetime

import logging
# import os
import time

class Timer:
    def __init__(self, ref):
        self.ref = ref

    def millis(self):
        return get_current_millis() - self.ref
    
def get_current_millis():
    return round(time.time() * 1000)

# def make_run_folder(folder):
#    today = datetime.today()
#    date = today.strftime("%Y-%m-%d_%H.%M.%S")
#    time = today.strftime("%H-%M%-S")
#    folder = f"debug/{date}-{time}"
#    path = os.path.join(os.getcwd(), folder, f"{date}-{time}")
#    
#    os.mkdir(path)
#    return path.join()

def get_filename():
    return datetime.today().strftime("%Y-%m-%d-%H.%M.%S")

def get_logger(folder, filename): 
    logging.basicConfig(filename=f"{folder}/{filename}.log", format="%(asctime)s %(levelname)s %(message)s", filemode="w") 
    logger = logging.getLogger() 
    logger.setLevel(logging.DEBUG)
    return logger
 
# logger.debug("Harmless debug Message")
# logger.info("Just an information")
# logger.warning("Its a Warning")
# logger.error("Did you try to divide by zero")
# logger.critical("Internet is down")