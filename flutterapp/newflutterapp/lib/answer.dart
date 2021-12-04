import 'package:flutter/material.dart';
class Answer extends StatelessWidget {
  final void Function() selectHandler;
  Answer(this.selectHandler);


  @override
  Widget build(BuildContext context) {
    return Container(
      width: double.infinity,
      child: ElevatedButton(
      onPressed: selectHandler,
      style: ElevatedButton.styleFrom(primary: Colors.blue,onPrimary: Colors.white),
      child: Text('answer 1')),
      
    );
  }
}