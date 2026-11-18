"""会话保存/加载契约。"""


def test_save_load_round_trip(store):
    session = store.create("测试")
    session.messages.append({"role": "user", "content": "你好"})
    store.save(session)

    loaded = store.load(session.session_id)
    assert loaded is not None
    assert loaded.messages == session.messages
    assert loaded.name == "测试"

    metas = store.list_sessions()
    assert [meta.name for meta in metas] == ["测试"]


def test_load_missing(store):
    assert store.load("missing") is None
