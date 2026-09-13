from pathlib import Path
import subprocess,re,json
r=Path(__file__).resolve().parents[1];d=json.loads((r/'docs/migration/pipe_micro_disposition.yaml').read_text());records=[]
for item in d['paths']:
 p=item['path']
 if not p.endswith(('.rs','.py','.json','.toml','.yml')):continue
 text=subprocess.check_output(['git','show',d['baseline_revision']+':'+p],cwd=r,text=True)
 entry={'path':p,'disposition':item['disposition'],'source_revision':d['baseline_revision'],'symbols':[],'numeric_source_lines':[]}
 for i,line in enumerate(text.splitlines(),1):
  if re.search(r'\b(fn|struct|enum|trait|const|def|class)\s+\w+',line):entry['symbols'].append({'line':i,'declaration':line.strip(),'status':'prior-library-domain-only' if item['disposition']=='keep' else 'retired-contract-or-recorded-property-gap'})
  if re.search(r'(?<![\w])\d+(?:\.\d+)?(?:e[-+]?\d+)?',line,re.I):entry['numeric_source_lines'].append({'line':i,'text':line.strip(),'status':'prior-library-domain-only' if item['disposition']=='keep' else 'retired'})
 if p.endswith('.json'):
  values=[]
  def walk(v,path):
   if isinstance(v,dict):
    for k,x in v.items():walk(x,path+'/'+k)
   elif isinstance(v,list):
    for k,x in enumerate(v):walk(x,path+'/'+str(k))
   else:values.append({'pointer':path,'value':v,'status':'historical-evidence-only' if item['disposition']=='archive' else 'retired'})
  walk(json.loads(text),'');entry['json_scalars']=values
 records.append(entry)
(r/'docs/migration/legacy_inventory.json').write_text(json.dumps({'schema':'pipe-micro-legacy-inventory/v1','scope':'Syntactic symbol/numeric-line inventory, not semantic proof of reuse admission','files':records},indent=2)+'\n')
