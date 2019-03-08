import random
filename = 'cdbw3_student_list.txt'
# 每組人數
num_in_one_group = 11
# 組序由 1 開始
group = 1
# 各班分組後所得數列
c2019 = []
with open(filename) as f:
    # 讀進全部檔案, 轉進數列, 並同時將各行的 \n 去除
    read_data = f.read().splitlines() 
print("共有 " + str(len(read_data)) + " 位學員")
# 利用 shuffle 將數列隨機弄亂
random.shuffle(read_data)
for i in range(len(read_data)):
    # 利用整數相除的餘數進行分組
    if i%num_in_one_group == 0:
        # 列出分隔符號
        print("-"*20)
        print("group " + str(group) +":")
        # 在分組區隔時重置各組學員數列
        group_list = []
        print()
        # 同時列出與分隔標註對應 i 的數列內容
        print(read_data[i])
        group_list.append(read_data[i])
        group = group + 1
    else:
        # 逐一列出同組的其他學員
        print(read_data[i])
        group_list.append(read_data[i])
    if i%num_in_one_group == 0:
        c2019.append(group_list)
# c2019 為該班分組後所得分組數列
print(c2019)