import 'package:flutter/material.dart';

import '../../core/models.dart';
import '../../theme.dart';
import 'panel.dart';

// VLA 的对话与判断。
//
// 分成两块是有意的：「判断」（模型这一轮怎么想）每轮都换，是调试信息；
// 「对话」（用户说了什么、狗回了什么）是要留着的，是交互记录。搅在一起的话，
// 想回看「我上一句说了什么」就得在两屏之间翻。
class ThoughtCard extends StatelessWidget {
  const ThoughtCard({super.key, required this.frame, this.maxLines = 6});

  final VlaFrame? frame;

  // 窄屏只有 150 px 给「判断」，六行会顶出去。
  final int maxLines;

  @override
  Widget build(BuildContext context) {
    final f = frame;
    final thought = f?.thought ?? '';
    final hasThought = thought.trim().isNotEmpty;
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        Row(
          children: [
            const StatusChip(text: '判断', color: C.vio, dense: true),
            const SizedBox(width: 8),
            // 这一行在手机上是「判断」卡片（约 216 pt），推理耗时那段文字
            // 原来不带弹性，414 pt 屏上溢出 87 px。让它自己截断。
            if (f != null)
              Flexible(
                child: Text(
                  '推理 ${f.genS.toStringAsFixed(2)}s · ${f.thinkRate.toStringAsFixed(2)} 次/s',
                  maxLines: 1,
                  overflow: TextOverflow.ellipsis,
                  style: const TextStyle(fontSize: 10.5, color: C.faint),
                ),
              ),
          ],
        ),
        const SizedBox(height: 8),
        Text(
          hasThought
              ? thought
              : (f?.emitThought == false ? '这一版不输出想法' : '还没想过'),
          maxLines: maxLines,
          overflow: TextOverflow.fade,
          style: TextStyle(
            fontSize: 13,
            height: 1.5,
            color: hasThought ? C.text : C.faint,
          ),
        ),
        if (f != null && (f.goal ?? '').isNotEmpty) ...[
          const SizedBox(height: 8),
          Row(
            children: [
              const Icon(Icons.flag_outlined, size: 12, color: C.faint),
              const SizedBox(width: 4),
              Expanded(
                child: Text(
                  '目标 ${f.goal}',
                  style: const TextStyle(fontSize: 11, color: C.dim),
                ),
              ),
            ],
          ),
        ],
      ],
    );
  }
}

// 视觉模型对画面的描述 + 它有多旧。
//
// 显示年龄是必要的：模型可能一直重复同一句，而那句话描述的是五秒前的画面。
// 只显示文字，人会以为「它现在看到的是这个」。
class SceneCard extends StatelessWidget {
  const SceneCard({super.key, required this.frame});
  final VlaFrame? frame;

  @override
  Widget build(BuildContext context) {
    final f = frame;
    final text = (f?.sceneText ?? '').trim();
    final age = f?.sceneAge;
    final stale = age != null && age > 4;
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        Row(
          children: [
            const StatusChip(text: '看到的', color: C.info, dense: true),
            const Spacer(),
            if (age != null)
              Text(
                '${age.toStringAsFixed(0)}s 前',
                style: TextStyle(
                  fontSize: 10.5,
                  color: stale ? C.warn : C.faint,
                ),
              ),
          ],
        ),
        const SizedBox(height: 8),
        Text(
          text.isEmpty ? '视觉模型还没给出描述' : text,
          maxLines: 4,
          overflow: TextOverflow.fade,
          style: TextStyle(
            fontSize: 12.5,
            height: 1.45,
            color: text.isEmpty ? C.faint : C.dim,
          ),
        ),
      ],
    );
  }
}

class ChatFeed extends StatefulWidget {
  const ChatFeed({super.key, required this.items, required this.rev});
  final List<ChatItem> items;
  final int rev;

  @override
  State<ChatFeed> createState() => _ChatFeedState();
}

class _ChatFeedState extends State<ChatFeed> {
  final _controller = ScrollController();

  @override
  void didUpdateWidget(ChatFeed old) {
    super.didUpdateWidget(old);
    if (widget.rev != old.rev) _scrollToEnd();
  }

  void _scrollToEnd() {
    WidgetsBinding.instance.addPostFrameCallback((_) {
      if (!_controller.hasClients) return;
      _controller.animateTo(
        _controller.position.maxScrollExtent,
        duration: const Duration(milliseconds: 220),
        curve: Curves.easeOut,
      );
    });
  }

  @override
  void dispose() {
    _controller.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    if (widget.items.isEmpty) {
      return const Center(
        child: Padding(
          padding: EdgeInsets.all(20),
          child: Text(
            '还没有对话。\n可以对它说「往前走」「你看到什么」「在我面前看着我」。',
            textAlign: TextAlign.center,
            style: TextStyle(fontSize: 11.5, color: C.faint, height: 1.6),
          ),
        ),
      );
    }
    return ListView.builder(
      controller: _controller,
      padding: const EdgeInsets.symmetric(vertical: 4),
      itemCount: widget.items.length,
      itemBuilder: (context, i) => _Bubble(item: widget.items[i]),
    );
  }
}

class _Bubble extends StatelessWidget {
  const _Bubble({required this.item});
  final ChatItem item;

  @override
  Widget build(BuildContext context) {
    final mine = item.mine;
    final color = mine ? C.accent : C.surfaceHi;
    return Align(
      alignment: mine ? Alignment.centerRight : Alignment.centerLeft,
      child: Container(
        margin: EdgeInsets.only(
          left: mine ? 28 : 0,
          right: mine ? 0 : 28,
          bottom: 6,
        ),
        padding: const EdgeInsets.symmetric(horizontal: 11, vertical: 8),
        decoration: BoxDecoration(
          color: mine ? color.withValues(alpha: 0.14) : color,
          borderRadius: BorderRadius.only(
            topLeft: const Radius.circular(11),
            topRight: const Radius.circular(11),
            bottomLeft: Radius.circular(mine ? 11 : 3),
            bottomRight: Radius.circular(mine ? 3 : 11),
          ),

        ),
        child: Column(
          crossAxisAlignment: mine
              ? CrossAxisAlignment.end
              : CrossAxisAlignment.start,
          children: [
            Text(
              item.text,
              style: TextStyle(
                fontSize: 12.5,
                height: 1.45,
                color: mine ? C.text : C.text,
              ),
            ),
            const SizedBox(height: 3),
            Text(
              '${item.mine ? '我' : '狗'}'
              '${item.repeat > 1 ? ' ×${item.repeat}' : ''} · '
              '${item.at.hour.toString().padLeft(2, '0')}:'
              '${item.at.minute.toString().padLeft(2, '0')}:'
              '${item.at.second.toString().padLeft(2, '0')}',
              style: const TextStyle(fontSize: 9.5, color: C.faint),
            ),
          ],
        ),
      ),
    );
  }
}
