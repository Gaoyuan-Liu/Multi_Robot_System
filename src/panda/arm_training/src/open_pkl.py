import pickle

f = open('final_Q_table.pkl', 'rb')
data = pickle.load(f)
print(data)
