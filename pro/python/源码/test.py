####################################################
#作者：wwk
#2020.5.25
#该程序用于转化实现数据为excle表格
#便于后期大量数据的
#直观统计和分析。
####################################################
#!/usr/bin/python
# -*- coding: UTF-8 -*-
import string
import csv

with open('./result.csv', 'w+', newline='') as csvfile:
    spamwriter = csv.writer(csvfile, dialect='excel')
    spamwriter.writerow(["图片名   ", "体长", "体宽", "臀宽", "体高"])
# 读要转换的txt文件，文件每行各词间以字符分隔]
    with open('../result/result.txt', 'r', encoding='utf-8') as filein:
        for line in filein:
            line_list = line.strip('\n').split(';')  # 我这里的数据之间是以 ; 间隔的
            img_name=line_list[0]
            name_list=img_name.split('/')  #利用/分割路径和命名
            if(name_list[0]=='..'):
                name=name_list[2]
                data_list=[name,line_list[2],line_list[4],line_list[6],line_list[8]]
                #print(data_list[0])
                spamwriter.writerow(data_list)
    print('''
            **************************
            ##########################
            *******转换完成！！*******
            ##########################
            **************************
            ''')
input()