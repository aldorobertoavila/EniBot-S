import pandas as pd

def convert_csv(path, sheet_name, csv):
    frame = pd.read_csv(csv)

    with pd.ExcelWriter(path) as writer:
        frame.columns = ['Time (ms)', 'Y']
        frame.to_excel(writer, index = None, header = True, sheet_name=sheet_name)