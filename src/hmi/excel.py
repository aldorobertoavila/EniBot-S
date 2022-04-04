import pandas as pd

def convert_csv(columns, path, csv):
    frame = pd.read_csv(csv)

    with pd.ExcelWriter(path) as writer:
        frame.columns = columns
        frame.to_excel(writer, index = None, header = True, sheet_name='Sensors')