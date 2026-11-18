"""应用容器：组装 settings / registry / agent，支持热加载。"""


from ..config import Settings
from ..skills import ToolRegistry
from ..skills.package_manager import PackageManager
from .agent import Agent, Approver
from .memory import MemoryStore, Session, SessionStore


class App:
    def __init__(
        self,
        settings: Settings,
        store: SessionStore,
        package_manager: PackageManager,
        approver: Approver | None = None,
    ):
        self.settings = settings
        self.store = store
        self.package_manager = package_manager
        self.approver = approver
        self.memory_store = (
            MemoryStore(settings.data_dir, namespace=settings.namespace)
            if settings.data_dir is not None
            else None
        )
        self.session: Session = store.create()
        self.registry = self._build_registry()
        self.agent = self._build_agent()

    def _build_registry(self) -> ToolRegistry:
        return ToolRegistry(
            self.settings.packages_dir,
            available=self.settings.packages or None,
            preload=self.settings.packages_preload,
            memory_searcher=self._memory_searcher,
            environment=self.settings.environment,
        )

    def _memory_searcher(self, query: str) -> str:
        if self.memory_store is None:
            return "长期记忆未启用"
        return self.memory_store.search_text(query, self.settings.memory_limit)

    def _build_agent(self) -> Agent:
        return Agent(
            self.settings.model,
            self.registry,
            self.settings,
            session=self.session,
            provider=self.settings.provider,
            approver=self.approver,
            memory_store=self.memory_store,
        )

    async def reload(self, settings: Settings) -> None:
        old_agent = self.agent
        self.settings = settings
        self.memory_store = (
            MemoryStore(self.settings.data_dir, namespace=self.settings.namespace)
            if self.settings.data_dir is not None
            else None
        )
        self.registry = self._build_registry()
        self.agent = self._build_agent()
        await old_agent.provider.close()
