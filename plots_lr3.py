import csv
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
plt.rcParams['font.family'] = 'DejaVu Sans'
METRICS = [
    ('mean_queue_wait','Среднее ожидание обслуженных заявок','queue_wait.png','Тики'),
    ('max_queue_wait','Максимальное ожидание заявки','max_wait.png','Тики'),
    ('completion_rate','Доля завершённых заказов','completion.png','Доля'),
    ('mean_delivery_ticks','Среднее время выполнения заказа','delivery.png','Тики'),
    ('queue_wait_jain','Равномерность ожидания (индекс Джейна)','fairness.png','Индекс [0; 1]'),
    ('emergency_recoveries','Аварийные восстановления','recoveries.png','Количество'),
]
def charts(folder='results_lr3'):
    out=Path(folder)
    with (out/'summary.csv').open(encoding='utf-8-sig',newline='') as f:
        rows=list(csv.DictReader(f))
    scenarios=['low','medium','high']
    for metric,title,filename,ylab in METRICS:
        fig,ax=plt.subplots(figsize=(7.2,4.3))
        for j,policy in enumerate(('fifo','priority')):
            values=[]
            for scenario in scenarios:
                value=next(r[metric+'_mean'] for r in rows if r['scenario']==scenario
                           and r['charge_policy']==policy)
                values.append(float(value) if value not in ('', None) else float('nan'))
            ax.bar([i + (j-.5)*.36 for i in range(3)],values,width=.35,
                   label=('FIFO' if policy=='fifo' else 'Приоритет'))
        ax.set_xticks(range(3),['Низкая','Средняя','Высокая'])
        ax.set_ylabel(ylab);ax.set_title(title);ax.legend();ax.grid(axis='y',alpha=.2)
        if metric in ('completion_rate','queue_wait_jain'):ax.set_ylim(0,1.05)
        fig.tight_layout();fig.savefig(out/filename,dpi=170);plt.close(fig)
if __name__=='__main__':charts()
