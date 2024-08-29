```mermaid
graph TD;
    A(RecordFromTopic.py) --> B(GetTagPose.py);
    B --> C(MergePose.py);
    C --> D(HandEyeCalib.py);
    B --> D(HandEyeCalib.py);

    

    
```