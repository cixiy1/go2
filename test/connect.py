from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.sport.sport_client import SportClient

ChannelFactoryInitialize(0, "en5")
sport_client = SportClient()
sport_client.SetTimeout(10.0)
sport_client.Init()
sport_client.Move(0.3,0,0)