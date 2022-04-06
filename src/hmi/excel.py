import pandas as pd

def convert_csv(columns, excel_file, csv_files, sheetnames):
    with pd.ExcelWriter(excel_file) as writer:
        for i, csv_file in enumerate(csv_files):
            frame = pd.read_csv(csv_file)
                
            frame.columns = columns
            frame.to_excel(writer, index = None, header = True, sheet_name=sheetnames[i])